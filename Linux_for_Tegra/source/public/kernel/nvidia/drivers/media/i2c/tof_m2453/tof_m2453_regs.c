// SPDX-License-Identifier: GPL-2.0
/*
 * tof_m2453_regs.c -- per-imager register maps.
 *
 * The SPI-flash bus master lives in different registers depending on the
 * imager.  Getting this wrong is not a "slightly wrong value", it is writing
 * into unrelated (sometimes reserved/analog) registers -- which is exactly
 * what happened on an IRS2877A, whose imager is an M2442 while the original
 * Python/flash.py code was written against the M2453/M2455/M2457 map.
 *
 * Maps taken verbatim from the royale SDK headers:
 *   source/components/imager/inc/imager/M2453/ImagerRegisters.hpp
 *   source/components/imager/inc/imager/M2457/ImagerRegisters.hpp
 *   source/components/imager/inc/imager/M2442/ImagerRegisters.hpp
 * and the bus masters:
 *   source/components/storage/src/SpiBusMasterM2453.cpp
 *   source/components/storage/src/SpiBusMasterM2442.cpp
 *
 * Note that doc/imager_dev_codes.md maps the sales codes:
 *   M2453 IRS2380C/IRS2381C, M2455 IRS2771C, M2457 IRS2877C,
 *   M2442 IRS2877A, M2458 IRS2977C, M2459 IRS2875C
 * A module called "IRS2877A" is an M2442, NOT an M2457 -- the two 2877
 * variants do not share the register map.
 */

#include <linux/string.h>

#include "tof_m2453.h"

/* M2453 / M2455 / M2457 (IRS2380C / IRS2771C / IRS2877C) */
const struct tof_imager_regs tof_regs_m2453 = {
	.name = "M2453/M2455/M2457",
	.spicfg = 0xA087,
	.spiwraddr = 0xA088,
	.spiraddr = 0xA089,
	.spilen = 0xA08A,
	.spitrig = 0xA08B,
	.spistatus = 0xA08C,

	/* SpiBusMasterM2453.cpp: 0x4002, i.e. without the polarity bit */
	.spicfg_polarity = false,

	.status = 0x9403,
	.status_idle_mask = 0x0001,
	.status_idle_value = 0x0001,

	.transmit_requires_read_enabled = false,
	.loads_usecase_from_flash = false,
	.pixels_dma_init = false,
};

/*
 * M2442 (IRS2877A).  Two things differ from the M2453 map:
 *  - the whole SPI block is shifted (SPICFG is at 0xA089, TRIG/STATUS at
 *    0xA08E/0xA08F, and the idle flag lives in STATUS0 at 0x940D -- 0x9403,
 *    which the M2453 map calls STATUS, is CFGCNT_SEQ_RECONFIG here);
 *  - the imager loads a use case register map *itself* from the module flash
 *    when it is told the 24 bit flash address, so there is no need to push
 *    1024 registers over I2C (see ImagerM2442.cpp:
 *    doFlashToImagerUseCaseTransfer()).
 */
const struct tof_imager_regs tof_regs_m2442 = {
	.name = "M2442 (IRS2877A)",
	.spicfg = 0xA089,
	.spiwraddr = 0xA08A,
	.spiraddr = 0xA08B,
	.spilen = 0xA08C,
	.spitrig = 0xA08E,
	.spistatus = 0xA08F,

	/* SpiBusMasterM2442.cpp: 0x5002, with the polarity bit */
	.spicfg_polarity = true,

	.status = 0x940D,			/* STATUS0 */
	.status_idle_mask = 0x0001,
	.status_idle_value = 0x0001,

	.transmit_requires_read_enabled = false,
	.loads_usecase_from_flash = true,
	.usecase_addr_hi = 0x9409,		/* USECASE_LOAD_ADDR0 */
	.usecase_addr_lo = 0x940A,		/* USECASE_LOAD_ADDR1 */
	.seq_mode = 0x9401,
	.trig = 0x9400,
	.usecase_load_error_mask = 0x0040,	/* STATUS0 bit 6 = load failed */
	.pixels_dma_init = true,
};

const struct tof_imager_regs *tof_m2453_regs_by_name(const char *name)
{
	if (!name)
		return NULL;
	/* the DT property usually carries the imager, the module or the sales
	 * code -- accept all the spellings that show up in practice */
	if (!strcasecmp(name, "m2442") || !strcasecmp(name, "irs2877a"))
		return &tof_regs_m2442;
	if (!strcasecmp(name, "m2453") || !strcasecmp(name, "m2455") ||
	    !strcasecmp(name, "m2457") || !strcasecmp(name, "irs2877c") ||
	    !strcasecmp(name, "irs2771c") || !strcasecmp(name, "irs2380c"))
		return &tof_regs_m2453;
	return NULL;
}
