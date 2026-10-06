// SPDX-License-Identifier: GPL-2.0
/*
 * tof_m2453_spi.c -- SPI bus master on the imager, module flash access.
 *
 * Port of tof_native/flash.py (class ImagerSpiFlash) plus the per-imager
 * byte-level differences taken from the royale sources.  The imager itself is
 * the SPI master: the host only pokes a handful of imager registers -- whose
 * addresses depend on the imager, see tof_regs() / tof_m2453_regs.c:
 *
 *   M2453/M2455/M2457 (IRS2877C):  0xA087 SPICFG,  0xA088 SPIWRADDR,
 *                                  0xA089 SPIRADDR, 0xA08A SPILEN,
 *                                  0xA08B SPITRIG,  0xA08C SPISTATUS
 *   M2442 (IRS2877A):              0xA089 SPICFG,  0xA08A SPIWRADDR,
 *                                  0xA08B SPIRADDR, 0xA08C SPILEN,
 *                                  0xA08E SPITRIG,  0xA08F SPISTATUS
 *
 * and pushes up to MAX_PREAMBLE(4) command bytes / pulls up to 256 data bytes
 * through the imager memory as 16 bit words.  A flash read is
 * "0x03, addr[23:0]" + N data bytes, so one 256 byte chunk costs one SPI
 * transfer, and a 2048 byte use-case block costs 8 of them.
 *
 * The M2442 additionally needs its pixel memory DMA-initialised before the bus
 * master may be used (M2442Utils.hpp: initBufferWithDma()).
 *
 * Python -> C map:
 *   open()                  -> tof_m2453_spi_open()
 *   transfer()              -> tof_spi_transfer()
 *   _wait_for_transfer()    -> tof_spi_wait()
 *   read()                  -> tof_m2453_flash_read()
 *   read_sequential_block() -> tof_m2453_flash_read_u16()
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <asm/unaligned.h>

#include "tof_m2453.h"

static int tof_spi_wait(struct tof_m2453 *tof);

/* Bring-up debugging aid, see TOF_SPI_REG_DELAY_US in the header. */
static void tof_spi_reg_gap(void)
{
#if TOF_SPI_REG_DELAY_US
	usleep_range(TOF_SPI_REG_DELAY_US, TOF_SPI_REG_DELAY_US + 1000);
#endif
}

/* write one SPI bus master register, with a log that names it on failure */
static int tof_spi_write_reg(struct tof_m2453 *tof, u16 addr, u16 value)
{
	int ret = tof_m2453_i2c_write_reg(tof, addr, value);

	if (ret)
		dev_err(tof->dev, "SPI reg 0x%04X = 0x%04X failed: %d\n",
			addr, value, ret);
	return ret;
}

/* Send `tx` (<= 4 bytes) and optionally receive `rx_len` bytes. */
static int tof_spi_transfer(struct tof_m2453 *tof, const u8 *tx, size_t tx_len,
			    u8 *rx, size_t rx_len)
{
	u16 *words = tof->word_buf;
	size_t total, words_len, wcount, i;
	u16 length;
	int ret;

	if (!tof->spi_open) {
		dev_err(tof->dev, "SPI transfer while the controller is closed\n");
		return -EPIPE;
	}
	if (tx_len == 0 || tx_len > TOF_SPI_MAX_PREAMBLE_BYTES) {
		dev_err(tof->dev, "invalid SPI command length %zu\n", tx_len);
		return -EINVAL;
	}
	if (rx_len > TOF_SPI_MAX_DATA_BYTES) {
		dev_err(tof->dev, "invalid SPI receive length %zu\n", rx_len);
		return -EINVAL;
	}
	total = tx_len + rx_len;
	if (total > TOF_SPI_MAX_PREAMBLE_BYTES + TOF_SPI_MAX_DATA_BYTES) {
		dev_err(tof->dev, "SPI command + payload too long (%zu)\n", total);
		return -EINVAL;
	}

	/*
	 * flash.py: the command/data bytes live in the imager memory as 16 bit
	 * words, so an odd command length gets one padding byte
	 * (pushBackIterableAsHighFirst16).  The SPILEN field below still uses
	 * the real command length.
	 */
	words_len = DIV_ROUND_UP(tx_len, 2);
	memset(words, 0, words_len * sizeof(*words));
	for (i = 0; i < tx_len; i++)
		words[i / 2] |= (u16)tx[i] << (i % 2 ? 0 : 8);

	ret = tof_m2453_i2c_write_burst(tof, TOF_SPI_WRITE_BUFFER_WORDS, words,
					words_len);
	if (ret) {
		dev_err(tof->dev, "cannot fill the SPI transmit buffer: %d\n", ret);
		return ret;
	}

	if (rx_len)
		length = ((u16)(tx_len - 1) << 14) | TOF_SPI_READ_ENABLE |
			 (u16)(total - 1);
	else if (tof_regs(tof)->transmit_requires_read_enabled)
		/* M2455 A11 workaround: read-enable is required even for a
		 * transmit-only transfer */
		length = TOF_SPI_READ_ENABLE | (u16)(total - 1);
	else
		length = (u16)(total - 1);

	ret = tof_spi_write_reg(tof, tof_regs(tof)->spiwraddr,
				TOF_SPI_WRITE_BUFFER_WORDS);
	if (ret)
		return ret;
	tof_spi_reg_gap();

	ret = tof_spi_write_reg(tof, tof_regs(tof)->spiraddr,
				TOF_SPI_READ_BUFFER_WORDS);
	if (ret)
		return ret;
	tof_spi_reg_gap();

	ret = tof_spi_write_reg(tof, tof_regs(tof)->spilen, length);
	if (ret)
		return ret;
	tof_spi_reg_gap();

	ret = tof_spi_write_reg(tof, tof_regs(tof)->spitrig,
				TOF_SPI_TRIGGER_VALUE);
	if (ret)
		return ret;

	ret = tof_spi_wait(tof);
	if (ret)
		return ret;
	if (!rx_len)
		return 0;

	wcount = DIV_ROUND_UP(rx_len, 2);
	ret = tof_m2453_i2c_read_burst(tof, TOF_SPI_READ_BUFFER_WORDS, words,
				       wcount);
	if (ret) {
		dev_err(tof->dev, "cannot read back the SPI receive buffer: %d\n",
			ret);
		return ret;
	}

	for (i = 0; i < wcount; i++) {
		u8 word[2];

		put_unaligned_be16(words[i], word);
		rx[2 * i] = word[0];
		if (2 * i + 1 < rx_len)
			rx[2 * i + 1] = word[1];
	}
	return 0;
}

/* flash.py _wait_for_transfer(): 500 us, then 10 ms steps, 31 tries. */
static int tof_spi_wait(struct tof_m2453 *tof)
{
	unsigned int attempt;

	for (attempt = 0; attempt <= TOF_SPI_MAX_POLL_RETRIES; attempt++) {
		u16 status = 0;
		int ret;

		if (attempt == 0)
			usleep_range(TOF_SPI_FIRST_WAIT_US,
				     TOF_SPI_FIRST_WAIT_US + 100);
		else
			msleep(TOF_SPI_POLL_INTERVAL_MS);

		ret = tof_m2453_i2c_read_reg(tof, tof_regs(tof)->spistatus,
					     &status);
		if (ret)
			return ret;
		if (status & TOF_SPI_ERROR_FLAG) {
			dev_err(tof->dev,
				"SPI hardware error flag set (SPISTATUS=0x%04X)\n",
				status);
			return -EIO;
		}
		if (status == TOF_SPI_DONE_VALUE)
			return 0;
	}

	dev_err(tof->dev, "SPI transfer did not complete in time\n");
	return -ETIMEDOUT;
}

/*
 * M2442 (IRS2877A) only: "the pixel memory has to initialized as this imager
 * checks the parity bits for each register and they are invalid after the
 * imager start up" (M2442Utils.hpp: initBufferWithDma()).  The SPI bus master
 * stores the transmit bytes in PIXMEM, so this has to run before the first
 * transfer -- and before SPICFG is enabled.
 */
static int tof_m2442_init_pixmem(struct tof_m2453 *tof)
{
	int ret, attempt;

	ret = tof_m2453_i2c_write_reg(tof, TOF_PIXMEM_DMA_SOURCE_REG, 0x0000);
	if (ret)
		return ret;
	ret = tof_m2453_i2c_write_reg(tof, TOF_PIXMEM_DMA_CFG, 0x0002);
	if (ret)
		return ret;
	ret = tof_m2453_i2c_write_reg(tof, TOF_PIXMEM_DMA_SRC,
				      TOF_PIXMEM_DMA_SOURCE_REG);
	if (ret)
		return ret;
	ret = tof_m2453_i2c_write_reg(tof, TOF_PIXMEM_DMA_DST, 0x0000);
	if (ret)
		return ret;
	ret = tof_m2453_i2c_write_reg(tof, TOF_PIXMEM_DMA_COUNT,
				      TOF_PIXMEM_DMA_WORDS);
	if (ret)
		return ret;
	ret = tof_m2453_i2c_write_reg(tof, TOF_PIXMEM_DMA_CTRL, 0x0001);
	if (ret)
		return ret;

	for (attempt = 0; attempt <= TOF_PIXMEM_DMA_POLL_RETRIES; attempt++) {
		u16 dma_status = 0;

		ret = tof_m2453_i2c_read_reg(tof, TOF_PIXMEM_DMA_STATUS,
					     &dma_status);
		if (ret)
			return ret;
		if (!(dma_status & 0x0001))
			return 0;
		usleep_range(TOF_PIXMEM_DMA_POLL_US, TOF_PIXMEM_DMA_POLL_US + 200);
	}

	dev_err(tof->dev, "PIXMEM DMA initialisation did not finish\n");
	return -ETIMEDOUT;
}

/*
 * flash.py open(): reset the imager, then enable the SPI controller.
 *
 * `polarity` is the 1<<12 clock polarity bit.  Pass tof_regs(tof)->spicfg_polarity
 * unless you have a reason not to: every imager that royale drives through an
 * SPI bus master except the M2453 sets it (M2453: 0x4002,
 * M2442/M2455/M2457/M2458/M2459/M4393: 0x5002), and without it the SPI reads
 * come back as zeros.
 */
int tof_m2453_spi_open(struct tof_m2453 *tof, bool polarity)
{
	u16 value = TOF_SPICFG_ENABLE | TOF_SPICFG_CLOCK_DIVISOR;
	int ret;

	// ret = tof_m2453_reset_pulse(tof);
	// if (ret)
	// 	return ret;

	/* M2442: the pixel memory must be parity-clean before the SPI bus
	 * master writes anything into it (M2442Utils.hpp). */
	if (tof_regs(tof)->pixels_dma_init) {
		ret = tof_m2442_init_pixmem(tof);
		if (ret)
			return ret;
	}

	if (polarity)
		value |= TOF_SPICFG_POLARITY;

	ret = tof_spi_write_reg(tof, tof_regs(tof)->spicfg, value);
	if (ret)
		return ret;

	tof->spi_open = true;
	return 0;
}

void tof_m2453_spi_close(struct tof_m2453 *tof)
{
	tof->spi_open = false;
}

/*
 * flash.py read(): 24 bit address, command 0x03 + 3 address bytes, at most
 * 256 data bytes per transfer.
 */
int tof_m2453_flash_read(struct tof_m2453 *tof, u32 address, void *dest,
			 size_t size)
{
	u8 *out = dest;
	size_t done = 0;

	if (!size)
		return 0;
	if (address >= TOF_FLASH_IMAGE_SIZE ||
	    size > TOF_FLASH_IMAGE_SIZE - address) {
		dev_err(tof->dev,
			"flash read of 0x%X+%zu is outside the 24 bit address space\n",
			address, size);
		return -EINVAL;
	}

	while (done < size) {
		size_t count = min_t(size_t, size - done, TOF_SPI_MAX_DATA_BYTES);
		u32 start = address + (u32)done;
		u8 command[4] = {
			TOF_FLASH_READ,
			(start >> 16) & 0xFF,
			(start >> 8) & 0xFF,
			start & 0xFF,
		};
		int ret;

		ret = tof_spi_transfer(tof, command, sizeof(command),
				       out + done, count);
		if (ret)
			return ret;
		done += count;
	}
	return 0;
}

/*
 * flash.py read_sequential_block(): a dense block of BIG endian 16 bit values.
 * `values` must have room for size / 2 entries.
 */
int tof_m2453_flash_read_u16(struct tof_m2453 *tof, u32 address, u16 *values,
			     size_t size)
{
	u8 *raw;
	size_t count, i;
	int ret;

	if (size % 2)
		return -EINVAL;
	if (!size)
		return 0;

	count = size / 2;
	raw = kmalloc(size, GFP_KERNEL);
	if (!raw)
		return -ENOMEM;

	ret = tof_m2453_flash_read(tof, address, raw, size);
	if (ret)
		goto out;

	for (i = 0; i < count; i++)
		values[i] = get_unaligned_be16(raw + i * 2);

out:
	kfree(raw);
	return ret;
}
