/* SPDX-License-Identifier: GPL-2.0 */
/*
 * tof_m2453.h -- Infineon ToF imager (M2453 / M2455 / M2457 / M2442):
 *                imager register + module SPI flash access.
 *
 * Ported from the userspace reference implementation in
 * tof_viewer_opt/tof_native/:
 *
 *   imager.py       ImagerM2453      register read/write, burst grouping,
 *                                    poll_until, reset, bring-up (init map)
 *   flash.py        ImagerSpiFlash   SPI bus master + flash read (0x03)
 *   zwe_parser.py   ZweParser        Zwetschge ToC / TableOfUseCases /
 *                                    TableOfRegisterMaps
 *   module_config.py ModuleConfig    turning the tables into register lists
 *                                    and caching the use-case register blocks
 *
 * plus the royale SDK sources for the pieces the Python does not cover:
 *   doc/imager_dev_codes.md, imager/inc/imager/<model>/ImagerRegisters.hpp,
 *   storage/src/SpiBusMaster{M2453,M2442}.cpp, imager/src/ImagerM2442.cpp
 *
 * Design: no userspace blob.  At probe time the driver reads only the small
 * tables plus -- for the imagers that need it -- the per-use-case dense
 * register blocks out of the module's SPI flash.  The ~200 kB calibration blob
 * is deliberately NOT read here; it belongs to userspace (the depth engine).
 *
 * ***** PER-IMAGER REGISTER MAP -- THE SPI BLOCK IS NOT THE SAME *****
 * The sales code is not the imager code (doc/imager_dev_codes.md):
 *
 *     IRS2380C/IRS2381C -> M2453      IRS2771C -> M2455
 *     IRS2877C          -> M2457      IRS2877A -> M2442
 *     IRS2977C          -> M2458      IRS2875C -> M2459
 *
 * The SPI bus master sits at 0xA087..0xA08C on the M2453/M2455/M2457 but at
 * 0xA089..0xA08F on the M2442.  Using the wrong table means writing reserved
 * or analog registers -- and an IRS2877A is an M2442, not an M2457.  Always
 * go through tof->regs / tof_regs(), see tof_m2453_regs.c.
 *
 * Byte order, this bites:
 *   - every field of the Zwetschge tables is LITTLE endian;
 *   - the imager sequential register blocks (flash -> imager) are BIG endian;
 *   - the imager I2C payload (register address and data) is BIG endian.
 * This mirrors the Python "<H" vs ">H" split in zwe_parser.py / imager.py.
 */
#ifndef __TOF_M2453_H
#define __TOF_M2453_H

#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/types.h>

/* ------------------------------------------------------------------ *
 * Imager registers that sit at the same address on every map
 * ------------------------------------------------------------------ */
#define TOF_REG_DESIGN_STEP		0xA0A5	/* ANAIP_DESIGNSTEP */
#define TOF_REG_PRODUCT_CODE		0xA0A4	/* ANAIP_PRODUCTCODE */
#define TOF_REG_CSI_HSIZE		0xA0A7	/* ANAIP_CSIHSIZE */
#define TOF_REG_CFGCNT_FLAGS		0x9402	/* M2453 only; unused here */
#define TOF_REG_START			0x9400	/* TRIG on all of them */

/* ------------------------------------------------------------------ *
 * SPI controller registers -- the M2453/M2455/M2457 addresses
 *
 * These are only the default map; the code uses tof_regs(tof)->*.  The M2442
 * (IRS2877A) has the whole block at 0xA089..0xA08F instead.
 * ------------------------------------------------------------------ */
#define TOF_REG_SPICFG			0xA087
#define TOF_REG_SPIWRADDR		0xA088
#define TOF_REG_SPIRADDR		0xA089
#define TOF_REG_SPILEN			0xA08A
#define TOF_REG_SPITRIG			0xA08B
#define TOF_REG_SPISTATUS		0xA08C

/* SPICFG value: enable the controller, divide the clock (the encoded field
 * value 2 means /8) and -- needed by the M2442/M2455/M2457/... but not by the
 * M2453, see SpiBusMasterM2453.cpp vs SpiBusMasterM2442.cpp -- set the clock
 * polarity bit.  Without the polarity bit the SPI reads return zeros. */
#define TOF_SPICFG_ENABLE		(1u << 14)
#define TOF_SPICFG_CLOCK_DIVISOR	2u
#define TOF_SPICFG_POLARITY		(1u << 12)

/* ------------------------------------------------------------------ *
 * SPILEN / SPISTATUS field values (flash.py)
 * ------------------------------------------------------------------ */
#define TOF_SPI_READ_ENABLE		(1u << 13)
#define TOF_SPI_TRIGGER_VALUE		2u
#define TOF_SPI_DONE_VALUE		1u
#define TOF_SPI_ERROR_FLAG		2u

#define TOF_SPI_WRITE_BUFFER_WORDS	0u	/* PIXMEM */
#define TOF_SPI_READ_BUFFER_WORDS	256u	/* PIXMEM + 256 (words) */

#define TOF_SPI_MAX_DATA_BYTES		256	/* == FLASH_PAGE_SIZE */
#define TOF_SPI_MAX_PREAMBLE_BYTES	4
#define TOF_SPI_REGISTER_SIZE		2
#define TOF_SPI_MAX_POLL_RETRIES	30
#define TOF_SPI_FIRST_WAIT_US		500
#define TOF_SPI_POLL_INTERVAL_MS	10

/*
 * Bring-up debugging aid: adds a 1..2 ms gap between the four SPI bus master
 * register writes (0 = off).  It was used while chasing a NACK right after the
 * reset pulse; the real cause there was the wrong imager register map, so it
 * is off by default.  Set it to 1 to reproduce that experiment.
 */
#define TOF_SPI_REG_DELAY_US		0

/* ------------------------------------------------------------------ *
 * Flash command set (SpiGenericFlash, WS25Q10EW)
 * ------------------------------------------------------------------ */
#define TOF_FLASH_READ			0x03
#define TOF_FLASH_IMAGE_SIZE		(1u << 24)	/* 24 bit addresses */

/* ------------------------------------------------------------------ *
 * Limits taken over from the Python / royale implementation
 * ------------------------------------------------------------------ */
/* MAXIMUM_DATA_SIZE(4096) / sizeof(u16) -- imager.py MAX_BURST_REGS */
#define TOF_MAX_BURST_REGS		2048
/* -> 2 address bytes + up to 2048 BE16 values in a single i2c message */
#define TOF_I2C_XFER_BUF_SIZE		(2 + (TOF_MAX_BURST_REGS * 2))
/* Default cap for one i2c message; Tegra adapters are usually 4096 total.
 * Board glue may lower/raise it before tof_m2453_probe_config(). */
#define TOF_I2C_DEFAULT_MAX_XFER	4096

/*
 * Optional retry for a register write that was answered with a NACK
 * (-EREMOTEIO / -ENXIO).  The USB bridge the Python reference talks to spaces
 * its accesses by milliseconds, so a slave that NACKs while it is busy never
 * shows up there; on a direct i2c bus it can.  Keep at 0 unless a NACK storm is
 * actually observed -- a retry also hides a wrong register map.
 */
#define TOF_I2C_NACK_RETRIES		0
#define TOF_I2C_NACK_RETRY_US		1000

/* polls for STOP-MAP: pollUntil(..., 500000 us), 1 ms interval */
#define TOF_STOP_POLL_TIMEOUT_MS	500
#define TOF_POLL_INTERVAL_US		1000

/* M2442 pixel memory DMA initialisation (M2442Utils.hpp) */
#define TOF_PIXMEM_DMA_POLL_RETRIES	10
#define TOF_PIXMEM_DMA_POLL_US		500
#define TOF_PIXMEM_DMA_SOURCE_REG	0x90BD	/* S63_EXPOTIME, the source */
#define TOF_PIXMEM_DMA_CFG		0x8634
#define TOF_PIXMEM_DMA_SRC		0x8635
#define TOF_PIXMEM_DMA_DST		0x8636
#define TOF_PIXMEM_DMA_COUNT		0x8637
#define TOF_PIXMEM_DMA_CTRL		0x8633
#define TOF_PIXMEM_DMA_STATUS		0x8638

/* M2442 pixel memory DMA: number of words transferred into PIXMEM */
#define TOF_PIXMEM_DMA_WORDS		0x0100

/* Zwetschge container */
#define TOF_ZWE_TOC_FLASH_ADDRESS	0x2000u
#define TOF_ZWE_TOC_READ_SIZE		137	/* TOC_V14C_SIZE, the maximum */
#define TOF_ZWE_TIMED_LIST_SLEEP_UNIT	32	/* sleep times are 32 us steps */

/* where the FW pages come from when they are sequential flash blocks; a
 * safety cap so a corrupt table cannot ask for a huge allocation. */
#define TOF_FW_BLOCK_MAX_BYTES		(64u * 1024u)

#define TOF_USECASE_NAME_MAX		32
#define TOF_MODULE_SERIAL_MAX		24
#define TOF_MODULE_NAME_MAX		20

/* ------------------------------------------------------------------ *
 * Per-imager register map
 * ------------------------------------------------------------------ */
struct tof_imager_regs {
	const char *name;

	/* SPI flash bus master (flash.py / SpiBusMaster*.cpp) */
	u16 spicfg;
	u16 spiwraddr;
	u16 spiraddr;
	u16 spilen;
	u16 spitrig;
	u16 spistatus;
	bool spicfg_polarity;		/* the 1<<12 bit of SPICFG */
	/* M2455 A11 needs the read-enable bit set even for pure writes */
	bool transmit_requires_read_enabled;

	/* polled after the STOP-MAP until the imager reports idle */
	u16 status;
	u16 status_idle_mask;
	u16 status_idle_value;

	/* the imager loads the use case register map itself from the module
	 * flash, given the 24 bit address of the block (M2442/IRS2877A) */
	bool loads_usecase_from_flash;
	u16 usecase_addr_hi;
	u16 usecase_addr_lo;
	u16 seq_mode;
	u16 trig;
	u16 usecase_load_error_mask;

	/* the pixel memory has to be DMA-initialised before the SPI bus master
	 * may be used: M2442, because the per-register parity bits are invalid
	 * after the imager has started up (M2442Utils.hpp) */
	bool pixels_dma_init;
};

extern const struct tof_imager_regs tof_regs_m2453;
extern const struct tof_imager_regs tof_regs_m2442;

const struct tof_imager_regs *tof_m2453_regs_by_name(const char *name);

/* ------------------------------------------------------------------ *
 * Descriptors
 * ------------------------------------------------------------------ */
struct tof_reg_write {
	u16 addr;
	u16 value;
	/* TimedRegisterList sleeps are stored in 32 us units, so the decoded
	 * value can exceed 16 bit (65535 * 32 us) -- keep it 32 bit wide. */
	u32 delay_us;
};

struct tof_reg_map {
	struct tof_reg_write *entries;
	unsigned int count;
};

/* The six TimedRegisterLists of TableOfRegisterMaps. */
struct tof_register_maps {
	struct tof_reg_map init;
	struct tof_reg_map fw_page_1;
	struct tof_reg_map fw_page_2;
	struct tof_reg_map fw_start;
	struct tof_reg_map start;
	struct tof_reg_map stop;
};

/* One use case: the metadata the driver needs plus -- for the imagers that
 * cannot load it themselves -- the cached dense register block. */
struct tof_usecase {
	char name[TOF_USECASE_NAME_MAX];
	char guid[40];
	u16 index;
	u16 width;
	u16 height;
	u8 raw_frames;
	u8 fps_start;
	u8 fps_min;
	u8 fps_max;
	/* SequentialRegisterHeader: where the block lives in the module flash,
	 * how big it is and which imager register it starts at. */
	u32 flash_addr;
	u32 flash_size;
	u16 imager_addr;
	/* cached copy of the block (BE16 values, `flash_size` bytes); NULL when
	 * the imager loads the map from the flash itself */
	u8 *block;
};

/* ------------------------------------------------------------------ *
 * Board glue
 * ------------------------------------------------------------------ */
struct tof_m2453;

struct tof_m2453_board_ops {
	/*
	 * Assert / release the imager reset line.  Required: the SPI controller
	 * bring-up starts with a reset pulse (flash.py: ImagerSpiFlash.open()
	 * -> ImagerM2453.reset_pulse()).
	 */
	int (*set_reset)(struct tof_m2453 *tof, bool asserted);
};

/* ------------------------------------------------------------------ *
 * Driver state
 * ------------------------------------------------------------------ */
struct tof_m2453 {
	struct device *dev;
	struct i2c_client *client;
	const struct tof_m2453_board_ops *ops;
	const struct tof_imager_regs *regs;	/* imager map, see above */
	struct mutex lock;		/* serialises I2C and flash access */

	/* module config read from the flash at probe */
	u32 toc_version;
	bool toc_crc_ok;
	char module_serial[TOF_MODULE_SERIAL_MAX];
	char module_name[TOF_MODULE_NAME_MAX];
	struct tof_register_maps maps;
	struct tof_usecase *use_cases;
	unsigned int num_use_cases;

	int active_usecase;		/* -1 == none written yet */
	bool spi_open;
	bool streaming;
	bool config_loaded;

	unsigned int max_xfer_bytes;	/* per i2c message, see above */
	bool verify_crc;

	/* scratch buffers, allocated once (no per-transfer allocation) */
	u8 *xfer_buf;			/* TOF_I2C_XFER_BUF_SIZE */
	u16 *word_buf;			/* TOF_MAX_BURST_REGS */
};

/* The map actually in use.  Board glue must set tof->regs before
 * tof_m2453_probe_config(); the fallback keeps older integrations working but
 * tof_m2453_probe_config() warns about it. */
static inline const struct tof_imager_regs *tof_regs(const struct tof_m2453 *tof)
{
	return tof->regs ? tof->regs : &tof_regs_m2453;
}

/* ------------------------------------------------------------------ *
 * i2c register access            (imager.py: ImagerM2453)
 * ------------------------------------------------------------------ */
int tof_m2453_i2c_read_reg(struct tof_m2453 *tof, u16 addr, u16 *value);
int tof_m2453_i2c_write_reg(struct tof_m2453 *tof, u16 addr, u16 value);
int tof_m2453_i2c_read_burst(struct tof_m2453 *tof, u16 first_addr,
			     u16 *values, unsigned int count);
int tof_m2453_i2c_write_burst(struct tof_m2453 *tof, u16 first_addr,
			      const u16 *values, unsigned int count);
/* hot path for a cached use-case block: BE16 stream -> consecutive registers */
int tof_m2453_i2c_write_be16_block(struct tof_m2453 *tof, u16 first_addr,
				   const u8 *be, size_t size);
int tof_m2453_poll_until(struct tof_m2453 *tof, u16 addr, u16 expected,
			 u16 mask, unsigned int timeout_ms);
int tof_m2453_reset_pulse(struct tof_m2453 *tof);

/* imager.py: transfer_register_map() -- burst grouping + delay handling */
int tof_m2453_transfer_register_map(struct tof_m2453 *tof,
				    const struct tof_reg_write *entries,
				    unsigned int count);

/* ------------------------------------------------------------------ *
 * SPI flash access               (flash.py: ImagerSpiFlash)
 * ------------------------------------------------------------------ */
int tof_m2453_spi_open(struct tof_m2453 *tof, bool polarity);
void tof_m2453_spi_close(struct tof_m2453 *tof);
int tof_m2453_flash_read(struct tof_m2453 *tof, u32 address, void *dest,
			 size_t size);
int tof_m2453_flash_read_u16(struct tof_m2453 *tof, u32 address, u16 *values,
			     size_t size);

/* ------------------------------------------------------------------ *
 * Module configuration           (zwe_parser.py + module_config.py)
 * ------------------------------------------------------------------ */
/* All of these take the driver lock. */
int tof_m2453_probe_config(struct tof_m2453 *tof);
void tof_m2453_free_config(struct tof_m2453 *tof);

/* Read every use case's dense register block from the flash and keep it in
 * RAM.  A no-op for the imagers that load the map from the flash themselves. */
int tof_m2453_cache_usecase_blocks(struct tof_m2453 *tof);

/* imager.py: initialize() + start_capture() + stop_capture() */
int tof_m2453_bring_up(struct tof_m2453 *tof);
int tof_m2453_start_capture(struct tof_m2453 *tof);
int tof_m2453_stop_capture(struct tof_m2453 *tof);

/*
 * imager.py: execute_usecase() -- writes the cached block, or hands the flash
 * address to the imager so it loads the map itself (M2442).
 *
 * This always writes.  The "re-selecting the active use case is a no-op"
 * shortcut it used to take is gone: active_usecase only records what this
 * driver last wrote, and the imager's registers can be wiped without the driver
 * noticing (tof_m2453_spi_open() starts with a reset pulse, and a power cycle
 * or a board level reset does the same).  In those cases the shortcut turned the
 * switch into a silent no-op -- "set_usecase sometimes does not reach the
 * sensor".  Callers that want to skip the traffic must skip the call.
 */
int tof_m2453_set_usecase(struct tof_m2453 *tof, unsigned int index);

/*
 * Same switch, but one i2c_transfer() per register instead of one message for
 * the whole 2 + N*2 byte block.
 *
 * Slower (1024 transfers for a 2048 byte block, where the burst needs one) and
 * the fallback for when the burst does not land on the imager: an adapter that
 * cannot send 2 + N*2 bytes in one message, or a slave that NACKs somewhere
 * inside the burst so only the leading part arrives.
 *
 * `send_zero_values` false leaves out the block entries whose value is 0x0000.
 * Writing them is what the reference does (ImagerM2453 pushes every entry), so
 * pass true unless there is a reason not to -- a skipped 0 also leaves whatever
 * the previous use case left in that register.
 *
 * Always writes, never takes the "already active" shortcut.  On failure it
 * clears active_usecase, because the imager then holds a mixture of the old and
 * the new block.
 */
int tof_m2453_set_usecase_reg(struct tof_m2453 *tof, unsigned int index,
			      bool send_zero_values);

int tof_m2453_find_usecase(struct tof_m2453 *tof, const char *name);

/* ImagerM2442.cpp: doFlashToImagerUseCaseTransfer() -- the imager fetches the
 * register map from the module flash itself (no 1024-register I2C burst).
 * The caller must hold tof->lock. */
int tof_m2453_load_usecase_from_flash(struct tof_m2453 *tof, u32 flash_addr);

/* Buffer + state helpers */
int tof_m2453_init_state(struct tof_m2453 *tof);
void tof_m2453_release(struct tof_m2453 *tof);

/* Internal, implemented in tof_m2453_zwe.c: read the ToC, the two small
 * tables and the register maps into tof->*. */
int tof_zwe_read_module_config(struct tof_m2453 *tof);

#endif /* __TOF_M2453_H */
