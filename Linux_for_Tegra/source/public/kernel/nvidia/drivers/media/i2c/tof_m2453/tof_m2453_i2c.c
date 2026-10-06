// SPDX-License-Identifier: GPL-2.0
/*
 * tof_m2453_i2c.c -- imager register access over plain i2c_transfer().
 *
 * Port of tof_native/imager.py (class ImagerM2453).  The Python original
 * reaches the imager registers through the Arctic USB vendor requests
 * (checked_get/checked_set on I2C_IMAGER); on the Jetson the same registers
 * sit on an I2C bus, so both collapse to
 *
 *     <16 bit register address, big endian><0..N big endian 16 bit values>
 *
 * Keep every byte-order decision in this file so the rest of the driver never
 * has to think about it.
 *
 * Python -> C map:
 *   write_register()          -> tof_m2453_i2c_write_reg()
 *   read_register()           -> tof_m2453_i2c_read_reg()
 *   write_burst()             -> tof_m2453_i2c_write_burst()
 *   read_burst()              -> tof_m2453_i2c_read_burst()
 *   transfer_register_map()   -> tof_m2453_transfer_register_map()
 *   poll_until()              -> tof_m2453_poll_until()
 *   reset_pulse()             -> tof_m2453_reset_pulse()
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <asm/unaligned.h>

#include "tof_m2453.h"

static int tof_i2c_xfer(struct tof_m2453 *tof, struct i2c_msg *msgs, int num)
{
	int ret;

	ret = i2c_transfer(tof->client->adapter, msgs, num);
	if (ret < 0)
		return ret;
	if (ret != num)
		return -EIO;
	return 0;
}

/* how many registers fit into one i2c message */
static unsigned int tof_i2c_regs_per_msg(struct tof_m2453 *tof)
{
	unsigned int regs;

	if (tof->max_xfer_bytes <= TOF_SPI_REGISTER_SIZE)
		return 0;
	regs = (tof->max_xfer_bytes - TOF_SPI_REGISTER_SIZE) / TOF_SPI_REGISTER_SIZE;
	return min(regs, (unsigned int)TOF_MAX_BURST_REGS);
}

int tof_m2453_i2c_read_reg(struct tof_m2453 *tof, u16 addr, u16 *value)
{
	u8 reg[2];
	u8 data[2];
	struct i2c_msg msg[2] = {
		{
			.addr = tof->client->addr,
			.flags = 0,
			.len = sizeof(reg),
			.buf = reg,
		},
		{
			.addr = tof->client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(data),
			.buf = data,
		},
	};
	int ret;

	put_unaligned_be16(addr, reg);
	ret = tof_i2c_xfer(tof, msg, 2);
	if (ret)
		return ret;
	*value = get_unaligned_be16(data);
	return 0;
}

int tof_m2453_i2c_write_reg(struct tof_m2453 *tof, u16 addr, u16 value)
{
	u8 buf[4];
	struct i2c_msg msg = {
		.addr = tof->client->addr,
		.flags = 0,
		.len = sizeof(buf),
		.buf = buf,
	};
	unsigned int attempt;
	int ret;

	put_unaligned_be16(addr, buf);
	put_unaligned_be16(value, buf + 2);

	for (attempt = 0; ; attempt++) {
		msg.buf = buf;
		msg.len = sizeof(buf);

		ret = tof_i2c_xfer(tof, &msg, 1);
		if (!ret)
			return 0;

		/* A busy slave may answer with a NACK; the retry is off unless
		 * TOF_I2C_NACK_RETRIES was raised (see tof_m2453.h) -- a silent
		 * retry would also hide a wrong register map. */
		if (attempt >= TOF_I2C_NACK_RETRIES ||
		    (ret != -EREMOTEIO && ret != -ENXIO))
			break;
		usleep_range(TOF_I2C_NACK_RETRY_US, TOF_I2C_NACK_RETRY_US + 100);
	}

	dev_err(tof->dev, "i2c write reg 0x%04X = 0x%04X failed: %d\n",
		addr, value, ret);
	return ret;
}

int tof_m2453_i2c_read_burst(struct tof_m2453 *tof, u16 first_addr,
			     u16 *values, unsigned int count)
{
	unsigned int regs_per_msg = tof_i2c_regs_per_msg(tof);
	unsigned int offset = 0;

	if (!regs_per_msg)
		return -EINVAL;

	/* imager.py read_burst(): chunk, then advance the address by the
	 * number of registers read so far. */
	while (offset < count) {
		unsigned int chunk = min(count - offset, regs_per_msg);
		u8 reg[2];
		struct i2c_msg msg[2] = {
			{
				.addr = tof->client->addr,
				.flags = 0,
				.len = sizeof(reg),
				.buf = reg,
			},
			{
				.addr = tof->client->addr,
				.flags = I2C_M_RD,
				.len = chunk * TOF_SPI_REGISTER_SIZE,
				.buf = tof->xfer_buf,
			},
		};
		unsigned int i;
		int ret;

		put_unaligned_be16((u16)(first_addr + offset), reg);
		ret = tof_i2c_xfer(tof, msg, 2);
		if (ret)
			return ret;

		for (i = 0; i < chunk; i++)
			values[offset + i] =
				get_unaligned_be16(tof->xfer_buf +
						   i * TOF_SPI_REGISTER_SIZE);
		offset += chunk;
	}
	return 0;
}

int tof_m2453_i2c_write_burst(struct tof_m2453 *tof, u16 first_addr,
			      const u16 *values, unsigned int count)
{
	unsigned int regs_per_msg = tof_i2c_regs_per_msg(tof);
	unsigned int offset = 0;

	if (!regs_per_msg)
		return -EINVAL;

	while (offset < count) {
		unsigned int chunk = min(count - offset, regs_per_msg);
		struct i2c_msg msg = {
			.addr = tof->client->addr,
			.flags = 0,
			.len = chunk * TOF_SPI_REGISTER_SIZE + 2,
			.buf = tof->xfer_buf,
		};
		unsigned int i;
		int ret;

		put_unaligned_be16((u16)(first_addr + offset), tof->xfer_buf);
		for (i = 0; i < chunk; i++)
			put_unaligned_be16(values[offset + i],
					   tof->xfer_buf + 2 + i * TOF_SPI_REGISTER_SIZE);

		ret = tof_i2c_xfer(tof, &msg, 1);
		if (ret)
			return ret;
		offset += chunk;
	}
	return 0;
}

/*
 * Same as write_burst() but the values are already a BE16 byte stream, as
 * read from the module flash.  This is the hot path for selecting a use case
 * (1024 registers / 2048 bytes for this module).
 */
int tof_m2453_i2c_write_be16_block(struct tof_m2453 *tof, u16 first_addr,
				   const u8 *be, size_t size)
{
	unsigned int regs_per_msg = tof_i2c_regs_per_msg(tof);
	size_t done = 0;

	if (!regs_per_msg)
		return -EINVAL;
	if (size % TOF_SPI_REGISTER_SIZE)
		return -EINVAL;

	while (done < size) {
		unsigned int regs = min_t(size_t, (size - done) / TOF_SPI_REGISTER_SIZE,
					  regs_per_msg);
		struct i2c_msg msg = {
			.addr = tof->client->addr,
			.flags = 0,
			.len = 2 + regs * TOF_SPI_REGISTER_SIZE,
			.buf = tof->xfer_buf,
		};
		int ret;

		put_unaligned_be16((u16)(first_addr + done / TOF_SPI_REGISTER_SIZE),
				   tof->xfer_buf);
		memcpy(tof->xfer_buf + 2, be + done, regs * TOF_SPI_REGISTER_SIZE);

		ret = tof_i2c_xfer(tof, &msg, 1);
		if (ret)
			return ret;
		done += (size_t)regs * TOF_SPI_REGISTER_SIZE;
	}
	return 0;
}

int tof_m2453_poll_until(struct tof_m2453 *tof, u16 addr, u16 expected,
			 u16 mask, unsigned int timeout_ms)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
	u16 last = 0;
	int ret;

	/* imager.py poll_until(): check, sleep 1 ms, check again, give up
	 * after the timeout. */
	for (;;) {
		ret = tof_m2453_i2c_read_reg(tof, addr, &last);
		if (ret)
			return ret;
		if ((last & mask) == (expected & mask))
			return 0;
		if (time_after_eq(jiffies, deadline))
			break;
		usleep_range(TOF_POLL_INTERVAL_US, TOF_POLL_INTERVAL_US + 500);
	}

	dev_err(tof->dev,
		"register 0x%04X: waited %u ms for 0x%04X (mask 0x%04X), last 0x%04X\n",
		addr, timeout_ms, expected & mask, mask, last);
	return -ETIMEDOUT;
}

int tof_m2453_reset_pulse(struct tof_m2453 *tof)
{
	int ret;

	if (!tof->ops || !tof->ops->set_reset)
		return -ENODEV;

	/* imager.py reset_pulse(): assert, sleep 1 ms, release, settle 10 ms. */
	ret = tof->ops->set_reset(tof, true);
	if (ret)
		return ret;
	usleep_range(1000, 2000);
	ret = tof->ops->set_reset(tof, false);
	if (ret)
		return ret;
	msleep(10);

	/*
	 * A reset wipes every register the imager was configured with, so the
	 * record of the selected use case is stale from here on.  Without this
	 * tof_m2453_set_usecase() could report success without writing anything
	 * (the "sometimes it does not get delivered" report) and
	 * tof_m2453_start_capture() would happily start on an unconfigured
	 * imager.
	 */
	tof->active_usecase = -1;
	tof->streaming = false;
	return 0;
}

/*
 * Port of imager.py transfer_register_map():
 *   - entries with a delay are written one by one followed by the sleep;
 *   - runs of consecutive addresses with zero delay are merged into a burst;
 *   - the group also ends when it would cross TOF_MAX_BURST_REGS.
 */
int tof_m2453_transfer_register_map(struct tof_m2453 *tof,
				    const struct tof_reg_write *entries,
				    unsigned int count)
{
	unsigned int i = 0;
	unsigned int bursts = 0;
	int ret;

	if (!entries || !count)
		return 0;

	while (i < count) {
		unsigned int j;

		if (entries[i].delay_us) {
			ret = tof_m2453_i2c_write_reg(tof, entries[i].addr,
						      entries[i].value);
			if (ret)
				return ret;
			/* the STOP-MAP delay is 500000 us; keep short ones tight */
			if (entries[i].delay_us > 10000)
				msleep(DIV_ROUND_UP(entries[i].delay_us, 1000));
			else
				usleep_range(entries[i].delay_us,
					     entries[i].delay_us + 100);
			i++;
			continue;
		}

		j = i + 1;
		while (j < count && !entries[j].delay_us &&
		       entries[j].addr == (u16)(entries[j - 1].addr + 1) &&
		       (unsigned int)(entries[j].addr - entries[i].addr) <
				TOF_MAX_BURST_REGS)
			j++;

		if (j - i == 1) {
			ret = tof_m2453_i2c_write_reg(tof, entries[i].addr,
						      entries[i].value);
		} else {
			unsigned int k;

			for (k = 0; k < j - i; k++)
				tof->word_buf[k] = entries[i + k].value;
			ret = tof_m2453_i2c_write_burst(tof, entries[i].addr,
							tof->word_buf, j - i);
			bursts++;
		}
		if (ret)
			return ret;
		i = j;
	}

	dev_dbg(tof->dev, "wrote %u registers (%u burst transfers)\n",
		count, bursts);
	return 0;
}
