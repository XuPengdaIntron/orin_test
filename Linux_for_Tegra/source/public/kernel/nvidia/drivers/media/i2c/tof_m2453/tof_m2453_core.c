// SPDX-License-Identifier: GPL-2.0
/*
 * tof_m2453_core.c -- module configuration caching + bring-up.
 *
 * Design: instead of loading a userspace-generated blob, probe reads the small
 * tables and then -- for the imagers that need it -- every use case's dense
 * register block straight out of the module's SPI flash and keeps it in RAM.
 * For the 850D module that is
 *
 *     137 B  ToC            (1 x 256 B transfer)
 *     172 B  TableOfRegisterMaps
 *     773 B  TableOfUseCases
 *   5 x 2048 B use case register blocks (8 x 256 B transfers each)
 *
 * i.e. 46 flash transfers / 10240 bytes.  That is a one-off cost at probe; how
 * long it takes depends on the i2c clock (the 256 B payload is pulled through
 * the imager memory over i2c): ~1.2 s at 100 kHz, ~0.3 s at 400 kHz, ~0.12 s
 * at 1 MHz.  The 211928 byte calibration blob is NOT touched -- it would be
 * 828 transfers on top of that.
 *
 * Imagers that load the register map themselves (M2442 = IRS2877A) skip the
 * block caching entirely and just get the 24 bit flash address of the block.
 *
 * Port of:
 *   module_config.read_module_config() / config_from_parser()
 *   module_config._sequential_registers()
 *   imager.ImagerM2453.initialize() / start_capture() / stop_capture()
 *   imager.ImagerM2453.execute_usecase()
 *   tof_camera.TofCamera.select_use_case() / start() / stop()
 *   ImagerM2442.cpp doFlashToImagerUseCaseTransfer()
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <asm/unaligned.h>

#include "tof_m2453.h"

/* imager.py VALID_DESIGN_STEPS (M2453 family) */
static const u16 tof_valid_design_steps[] = { 0x0A11, 0x0A12, 0x0B11, 0x0B12 };

/* ------------------------------------------------------------------ *
 * state / buffers
 * ------------------------------------------------------------------ */
int tof_m2453_init_state(struct tof_m2453 *tof)
{
	if (!tof->dev || !tof->client)
		return -EINVAL;

	mutex_init(&tof->lock);
	tof->active_usecase = -1;
	tof->streaming = false;
	tof->spi_open = false;
	tof->config_loaded = false;

	/* Callers may override these two after init_state() and before
	 * tof_m2453_probe_config(). */
	if (!tof->max_xfer_bytes)
		tof->max_xfer_bytes = TOF_I2C_DEFAULT_MAX_XFER;
	tof->verify_crc = true;

	tof->xfer_buf = devm_kmalloc(tof->dev, TOF_I2C_XFER_BUF_SIZE, GFP_KERNEL);
	tof->word_buf = devm_kmalloc_array(tof->dev, TOF_MAX_BURST_REGS,
					   sizeof(*tof->word_buf), GFP_KERNEL);
	if (!tof->xfer_buf || !tof->word_buf)
		return -ENOMEM;

	if (!tof->regs)
		dev_warn(tof->dev,
			 "no imager register map selected; assuming %s (set tof->regs, see tof_m2453_regs.c)\n",
			 tof_regs_m2453.name);
	else
		dev_info(tof->dev, "imager register map: %s\n", tof->regs->name);

	return 0;
}

static void free_reg_map(struct tof_reg_map *map)
{
	kvfree(map->entries);
	map->entries = NULL;
	map->count = 0;
}

void tof_m2453_free_config(struct tof_m2453 *tof)
{
	unsigned int i;

	for (i = 0; i < tof->num_use_cases; i++)
		kfree(tof->use_cases[i].block);
	tof->num_use_cases = 0;
	kfree(tof->use_cases);
	tof->use_cases = NULL;

	free_reg_map(&tof->maps.init);
	free_reg_map(&tof->maps.fw_page_1);
	free_reg_map(&tof->maps.fw_page_2);
	free_reg_map(&tof->maps.fw_start);
	free_reg_map(&tof->maps.start);
	free_reg_map(&tof->maps.stop);

	tof->config_loaded = false;
}

void tof_m2453_release(struct tof_m2453 *tof)
{
	mutex_lock(&tof->lock);
	tof_m2453_free_config(tof);
	mutex_unlock(&tof->lock);
}

/* ------------------------------------------------------------------ *
 * probe: read the tables + cache the use case register blocks
 * ------------------------------------------------------------------ */
int tof_m2453_cache_usecase_blocks(struct tof_m2453 *tof)
{
	unsigned int i;
	size_t cached = 0;
	ktime_t start;
	int ret;

	if (!tof->use_cases || !tof->num_use_cases)
		return 0;

	/*
	 * The M2442 fetches the register map from the flash itself when it is
	 * told the address (doFlashToImagerUseCaseTransfer), so caching 5 x
	 * 2048 bytes would only cost RAM and probe time.
	 */
	if (tof_regs(tof)->loads_usecase_from_flash) {
		for (i = 0; i < tof->num_use_cases; i++) {
			struct tof_usecase *uc = &tof->use_cases[i];

			dev_info(tof->dev,
				 "  [%u] %-24s %ux%u raw=%u fps %u/%u-%u  load from flash 0x%06X -> imager 0x%04X\n",
				 i, uc->name, uc->width, uc->height, uc->raw_frames,
				 uc->fps_start, uc->fps_min, uc->fps_max,
				 uc->flash_addr, uc->imager_addr);
		}
		dev_info(tof->dev,
			 "%s loads the use case register maps itself; no blocks cached\n",
			 tof_regs(tof)->name);
		return 0;
	}

	start = ktime_get();

	for (i = 0; i < tof->num_use_cases; i++) {
		struct tof_usecase *uc = &tof->use_cases[i];

		if (!uc->flash_size || !uc->imager_addr) {
			dev_warn(tof->dev,
				 "use case %s has no register block, skipped\n",
				 uc->name);
			continue;
		}
		if (uc->flash_size % TOF_SPI_REGISTER_SIZE) {
			dev_warn(tof->dev,
				 "use case %s: odd block size %u, skipped\n",
				 uc->name, uc->flash_size);
			continue;
		}

		uc->block = kmalloc(uc->flash_size, GFP_KERNEL);
		if (!uc->block) {
			ret = -ENOMEM;
			goto err;
		}

		ret = tof_m2453_flash_read(tof, uc->flash_addr, uc->block,
					   uc->flash_size);
		if (ret) {
			dev_err(tof->dev,
				"cannot read the register block of %s: %d\n",
				uc->name, ret);
			kfree(uc->block);
			uc->block = NULL;
			goto err;
		}

		cached += uc->flash_size;
		dev_info(tof->dev,
			 "  [%u] %-24s %ux%u raw=%u fps %u/%u-%u  block 0x%X+%u -> imager 0x%04X\n",
			 i, uc->name, uc->width, uc->height, uc->raw_frames,
			 uc->fps_start, uc->fps_min, uc->fps_max,
			 uc->flash_addr, uc->flash_size, uc->imager_addr);
	}

	dev_info(tof->dev,
		 "cached %u use case register block(s), %zu bytes, in %lld ms\n",
		 tof->num_use_cases, cached,
		 ktime_ms_delta(ktime_get(), start));
	return 0;

err:
	/* keep what was cached so far; the caller decides whether to fail */
	return ret;
}

int tof_m2453_probe_config(struct tof_m2453 *tof)
{
	ktime_t start;
	int ret;

	if (!tof->regs)
		dev_warn(tof->dev,
			 "no imager register map selected; assuming %s -- the SPI block is at different addresses on the M2442 (IRS2877A)!\n",
			 tof_regs_m2453.name);

	mutex_lock(&tof->lock);

	if (tof->config_loaded) {
		ret = 0;
		goto out;
	}

	start = ktime_get();

	/* module_config.read_module_config(): the imager is reset while the
	 * flash is read, so this must run before the imager bring-up. */
	ret = tof_m2453_spi_open(tof, tof_regs(tof)->spicfg_polarity);
	if (ret) {
		dev_err(tof->dev, "cannot enable the imager SPI controller: %d\n",
			ret);
		goto out;
	}

	ret = tof_zwe_read_module_config(tof);
	if (ret) {
		dev_err(tof->dev, "tof_zwe_read_module_config: %d\n", ret);
		goto out;
	}

	ret = tof_m2453_cache_usecase_blocks(tof);
	if (ret) {
		dev_err(tof->dev, "tof_m2453_cache_usecase_blocks: %d\n", ret);
		goto out;
	}

	tof->config_loaded = true;
	dev_info(tof->dev, "module configuration read in %lld ms\n",
		 ktime_ms_delta(ktime_get(), start));
out:
	mutex_unlock(&tof->lock);
	return ret;
}

/* ------------------------------------------------------------------ *
 * imager bring-up (imager.py initialize/start/stop)
 * ------------------------------------------------------------------ */
int tof_m2453_bring_up(struct tof_m2453 *tof)
{
	u16 design_step = 0;
	unsigned int i;
	bool known = false;
	ktime_t start;
	int ret;

	mutex_lock(&tof->lock);
	start = ktime_get();

	if (!tof->config_loaded) {
		ret = -EINVAL;
		goto out;
	}

	// ret = tof_m2453_reset_pulse(tof);
	// if (ret) {
	// 	dev_err(tof->dev, "reset pulse failed: %d\n", ret);
	// 	goto out;
	// }

	ret = tof_m2453_i2c_read_reg(tof, TOF_REG_DESIGN_STEP, &design_step);
	if (ret) {
		dev_err(tof->dev,
			"cannot read the design step register (0x%04X): %d (power/reset/clock?)\n",
			TOF_REG_DESIGN_STEP, ret);
		goto out;
	}
	for (i = 0; i < ARRAY_SIZE(tof_valid_design_steps); i++) {
		if (design_step == tof_valid_design_steps[i])
			known = true;
	}

	/*
	 * ANAIP_PRODUCTCODE / ANAIP_CSIHSIZE sit at the same addresses on the
	 * M2453 and the M2442 maps and are the quickest way to see which imager
	 * is really on the bus when a register map is in doubt.
	 */
	{
		u16 product = 0, hsize = 0;

		if (!tof_m2453_i2c_read_reg(tof, TOF_REG_PRODUCT_CODE, &product) &&
		    !tof_m2453_i2c_read_reg(tof, TOF_REG_CSI_HSIZE, &hsize))
			dev_info(tof->dev,
				 "design step 0x%04X, product code 0x%04X, CSI hsize 0x%04X (%s)\n",
				 design_step, product, hsize, tof_regs(tof)->name);
		else
			dev_info(tof->dev, "design step 0x%04X (%s)\n",
				 design_step, tof_regs(tof)->name);
	}

	/* only the M2453 design steps are enumerated here; for the other
	 * imagers just report the value */
	if (tof_regs(tof) == &tof_regs_m2453 && !known)
		dev_warn(tof->dev,
			 "unexpected design step 0x%04X for %s, continuing anyway\n",
			 design_step, tof_regs(tof)->name);

	/* INIT-MAP -> FW-PAGE-1 -> FW-PAGE-2 -> FW-START-MAP, skipping the
	 * pages this module keeps in its own flash (empty lists here). */
	ret = tof_m2453_transfer_register_map(tof, tof->maps.init.entries,
					      tof->maps.init.count);
	if (ret)
	{
		dev_err(tof->dev, "tof_m2453_transfer_register_map(init) failed: %d\n", ret);
		goto out;
	}
		
	ret = tof_m2453_transfer_register_map(tof, tof->maps.fw_page_1.entries,
					      tof->maps.fw_page_1.count);
	if (ret)
	{
		dev_err(tof->dev, "tof_m2453_transfer_register_map(fw_page_1) failed: %d\n", ret);
		goto out;
	}
	ret = tof_m2453_transfer_register_map(tof, tof->maps.fw_page_2.entries,
					      tof->maps.fw_page_2.count);
	if (ret)
	{
		dev_err(tof->dev, "tof_m2453_transfer_register_map(fw_page_2) failed: %d\n", ret);
		goto out;
	}
	ret = tof_m2453_transfer_register_map(tof, tof->maps.fw_start.entries,
					      tof->maps.fw_start.count);
	if (ret)
	{
		dev_err(tof->dev, "tof_m2453_transfer_register_map(fw_start) failed: %d\n", ret);
		goto out;
	}

	tof->active_usecase = -1;
	dev_info(tof->dev, "bring-up done in %lld ms\n",
		 ktime_ms_delta(ktime_get(), start));
out:
	mutex_unlock(&tof->lock);
	return ret;
}

int tof_m2453_start_capture(struct tof_m2453 *tof)
{
	int ret;

	mutex_lock(&tof->lock);

	if (!tof->config_loaded) {
		ret = -EINVAL;
		goto out;
	}
	/* tof_camera.TofCamera.start(): a use case must have been selected */
	if (tof->active_usecase < 0) {
		dev_err(tof->dev, "no use case selected, refusing to start\n");
		ret = -EINVAL;
		goto out;
	}

	ret = tof_m2453_transfer_register_map(tof, tof->maps.start.entries,
					      tof->maps.start.count);
	if (ret)
		goto out;

	tof->streaming = true;
	dev_info(tof->dev, "START sent (0x%04X=0x0001)\n", TOF_REG_START);
out:
	mutex_unlock(&tof->lock);
	return ret;
}

int tof_m2453_stop_capture(struct tof_m2453 *tof)
{
	int ret;

	mutex_lock(&tof->lock);

	if (!tof->streaming) {
		ret = 0;
		goto out;
	}

	/* STOP-MAP writes 0x9400 = 0 with a 500 ms delay; the following poll
	 * waits for the status register to report idle again
	 * (M2453: 0x9403 bit 0, M2442: STATUS0 0x940D bit 0). */
	ret = tof_m2453_transfer_register_map(tof, tof->maps.stop.entries,
					      tof->maps.stop.count);
	if (ret)
		goto out;

	/*
	 * The imager has been told to stop, so clear the flag *before* the poll:
	 * clearing it only after a successful poll means a poll timeout (500 ms
	 * is short, and the idle bit lives at a different address on the M2442)
	 * left streaming == true forever.  Every later tof_m2453_set_usecase()
	 * then answered -EBUSY without ever touching the sensor -- another way
	 * for a switch to "not happen".
	 */
	tof->streaming = false;

	ret = tof_m2453_poll_until(tof, tof_regs(tof)->status,
				   tof_regs(tof)->status_idle_value,
				   tof_regs(tof)->status_idle_mask,
				   TOF_STOP_POLL_TIMEOUT_MS);
	if (ret) {
		dev_warn(tof->dev,
			 "STOP was sent but the imager did not report idle; the switch is allowed again, check the status register (0x%04X)\n",
			 tof_regs(tof)->status);
		goto out;
	}

	dev_info(tof->dev, "STOP sent, status idle\n");
out:
	mutex_unlock(&tof->lock);
	return ret;
}

/* ------------------------------------------------------------------ *
 * use case selection (imager.py execute_usecase)
 * ------------------------------------------------------------------ */

/* Validation shared by both writers.  The caller must hold tof->lock. */
static int tof_check_usecase(struct tof_m2453 *tof, unsigned int index,
			     struct tof_usecase **uc_out)
{
	struct tof_usecase *uc;

	if (!tof->config_loaded) {
		dev_err(tof->dev, "module configuration not read, nothing to switch\n");
		return -EINVAL;
	}
	if (index >= tof->num_use_cases) {
		dev_err(tof->dev, "use case %u does not exist (%u available)\n",
			index, tof->num_use_cases);
		return -EINVAL;
	}

	uc = &tof->use_cases[index];

	/* tof_camera.TofCamera.select_use_case() refuses to switch while
	 * streaming; the firmware must not be reconfigured under a live
	 * frame stream. */
	if (tof->streaming) {
		dev_err(tof->dev, "stop capture before switching use case\n");
		return -EBUSY;
	}
	/* imagers that fetch the map themselves need no cached block */
	if (!uc->block && !tof_regs(tof)->loads_usecase_from_flash) {
		dev_err(tof->dev, "use case %s was not cached\n", uc->name);
		return -ENODATA;
	}

	*uc_out = uc;
	return 0;
}

/*
 * Push the cached block, either as one i2c message (the normal path) or one
 * transfer per register (the fallback / the zero-skipping path).
 */
static int tof_write_usecase_block(struct tof_m2453 *tof,
				   const struct tof_usecase *uc, bool per_reg,
				   bool send_zero_values)
{
	if (!uc->block) {
		dev_err(tof->dev, "use case %s was not cached\n", uc->name);
		return -ENODATA;
	}

	if (!per_reg) {
		int ret = tof_m2453_i2c_write_be16_block(tof, uc->imager_addr,
							 uc->block,
							 uc->flash_size);

		if (ret)
			dev_err(tof->dev,
				"cannot write the %s register block: %d\n",
				uc->name, ret);
		return ret;
	}

	{
		unsigned int count = uc->flash_size / TOF_SPI_REGISTER_SIZE;
		unsigned int i, sent = 0, skipped = 0;
		ktime_t start = ktime_get();

		for (i = 0; i < count; i++) {
			u16 value = get_unaligned_be16(uc->block +
						       i * TOF_SPI_REGISTER_SIZE);
			int ret;

			if (!value && !send_zero_values) {
				skipped++;
				continue;
			}

			ret = tof_m2453_i2c_write_reg(tof,
						      (u16)(uc->imager_addr + i),
						      value);
			if (ret) {
				dev_err(tof->dev,
					"use case %s: register 0x%04X = 0x%04X failed (%u of %u written so far): %d\n",
					uc->name,
					(unsigned int)(uc->imager_addr + i),
					value, sent, count, ret);
				/* the imager now holds a mixture of the old and
				 * the new block, so no use case is active */
				tof->active_usecase = -1;
				return ret;
			}
			sent++;
			if ((sent % 256) == 0)
				dev_dbg(tof->dev, "  %u/%u registers written\n",
					sent, count);
		}

		dev_info(tof->dev,
			 "use case %s: %u register(s) written one by one, %u zero value(s) %s, in %lld ms\n",
			 uc->name, sent, skipped,
			 send_zero_values ? "included" : "skipped",
			 ktime_ms_delta(ktime_get(), start));
		return 0;
	}
}

int tof_m2453_set_usecase(struct tof_m2453 *tof, unsigned int index)
{
	struct tof_usecase *uc;
	ktime_t start;
	int ret;

	mutex_lock(&tof->lock);

	ret = tof_check_usecase(tof, index, &uc);
	if (ret)
		goto out;

	/*
	 * There used to be a shortcut here ("idempotent: re-selecting the active
	 * use case is a no-op").  It is unsound and it is the usual reason a
	 * switch never reaches the sensor: active_usecase only records what this
	 * driver last wrote, while the imager's registers can be gone for reasons
	 * it never sees --
	 *
	 *   - tof_m2453_spi_open() starts with tof_m2453_reset_pulse(),
	 *   - a power cycle or a board level (GPIO/watchdog) reset,
	 *   - the boot loader or another agent configuring the imager,
	 *   - a previous write that failed halfway.
	 *
	 * In each of those cases active_usecase still named the old index and the
	 * call became a silent no-op.  So it is a record now, never a gate: the
	 * block is always pushed.  Callers that want to skip the traffic must
	 * skip the call.
	 */
	if (tof->active_usecase == (int)index)
		dev_dbg(tof->dev,
			"use case %s is already recorded as active, writing it again\n",
			uc->name);

	start = ktime_get();

	if (tof_regs(tof)->loads_usecase_from_flash) {
		ret = tof_m2453_load_usecase_from_flash(tof, uc->flash_addr);
		if (ret) {
			dev_err(tof->dev,
				"cannot load the %s register map from flash 0x%X: %d\n",
				uc->name, uc->flash_addr, ret);
			/* do not leave a wrong index behind: the next
			 * set_usecase() must write again, and start_capture()
			 * must refuse */
			tof->active_usecase = -1;
			goto out;
		}
		dev_info(tof->dev,
			 "use case %s: loaded from flash 0x%06X in %lld ms\n",
			 uc->name, uc->flash_addr,
			 ktime_ms_delta(ktime_get(), start));
	} else {
		ret = tof_write_usecase_block(tof, uc, false, false);
		if (ret) {
			tof->active_usecase = -1;
			goto out;
		}
		dev_info(tof->dev,
			 "use case %s: %u registers written in %lld ms\n",
			 uc->name, uc->flash_size / TOF_SPI_REGISTER_SIZE,
			 ktime_ms_delta(ktime_get(), start));
	}

	tof->active_usecase = index;
out:
	mutex_unlock(&tof->lock);
	return ret;
}

/*
 * The register-by-register variant of tof_m2453_set_usecase(): one i2c
 * transfer per register, optionally leaving out the entries whose value is 0.
 *
 * Use it when the single-message burst does not reach the imager -- an adapter
 * that cannot send 2 + N*2 bytes in one message, or a slave that NACKs inside
 * the burst so only the leading part arrives.  It is the "make sure it lands"
 * path: it always writes, and it is the only way to leave the 0x0000 entries
 * out.
 */
int tof_m2453_set_usecase_reg(struct tof_m2453 *tof, unsigned int index,
			      bool send_zero_values)
{
	struct tof_usecase *uc;
	ktime_t start;
	int ret;

	mutex_lock(&tof->lock);

	ret = tof_check_usecase(tof, index, &uc);
	if (ret)
		goto out;

	if (tof_regs(tof)->loads_usecase_from_flash) {
		/* Nothing to push: the imager fetches the map from the module
		 * flash itself, so writing registers to it would be the wrong
		 * mechanism entirely (that is the M2442/IRS2877A). */
		dev_err(tof->dev,
			"%s loads the use case register map itself; use tof_m2453_set_usecase()\n",
			tof_regs(tof)->name);
		ret = -EOPNOTSUPP;
		goto out;
	}

	start = ktime_get();
	ret = tof_write_usecase_block(tof, uc, true, send_zero_values);
	if (ret)
		goto out;		/* active_usecase was cleared already */

	tof->active_usecase = index;
	dev_info(tof->dev,
		 "use case %s: register-by-register write complete in %lld ms\n",
		 uc->name, ktime_ms_delta(ktime_get(), start));
out:
	mutex_unlock(&tof->lock);
	return ret;
}

/*
 * ImagerM2442.cpp: doFlashToImagerUseCaseTransfer().  The M2442 does not get
 * the register map pushed into it; it reads the (dense) block from the module
 * flash itself, so all the host has to hand over is its 24 bit flash address.
 * The caller must hold tof->lock.
 */
int tof_m2453_load_usecase_from_flash(struct tof_m2453 *tof, u32 flash_addr)
{
	u16 prev_mode = 0;
	u16 status = 0;
	int ret;

	if (!tof_regs(tof)->loads_usecase_from_flash)
		return -EOPNOTSUPP;
	if (flash_addr > 0xFFFFFF)
		return -EINVAL;

	/* "Lower the SPI clock to make the transfer more reliable": enable the
	 * SPI controller (clock divisor 8) so the imager can read its flash. */
	ret = tof_m2453_i2c_write_reg(tof, tof_regs(tof)->spicfg,
				      TOF_SPICFG_ENABLE | TOF_SPICFG_POLARITY |
				      TOF_SPICFG_CLOCK_DIVISOR);
	if (ret)
		return ret;
	usleep_range(500, 700);

	/* Tell the imager where the configuration can be found. */
	ret = tof_m2453_i2c_write_reg(tof, tof_regs(tof)->usecase_addr_hi,
				      (u16)((flash_addr >> 16) & 0x00FF));
	if (ret)
		return ret;
	ret = tof_m2453_i2c_write_reg(tof, tof_regs(tof)->usecase_addr_lo,
				      (u16)(flash_addr & 0xFFFF));
	if (ret)
		return ret;

	/* Save the current mode, switch to "load use case" mode. */
	ret = tof_m2453_i2c_read_reg(tof, tof_regs(tof)->seq_mode, &prev_mode);
	if (ret)
		return ret;
	ret = tof_m2453_i2c_write_reg(tof, tof_regs(tof)->seq_mode, 0x0007);
	if (ret)
		return ret;
	usleep_range(500, 700);

	/* Start the loading function by triggering the imager. */
	ret = tof_m2453_i2c_write_reg(tof, tof_regs(tof)->trig, 0x0001);
	if (ret)
		return ret;
	msleep(10);
	ret = tof_m2453_i2c_write_reg(tof, tof_regs(tof)->trig, 0x0000);
	if (ret)
		return ret;

	/* Check the status bit of the use case loading function. */
	ret = tof_m2453_i2c_read_reg(tof, tof_regs(tof)->status, &status);
	if (ret)
		return ret;
	if (status & tof_regs(tof)->usecase_load_error_mask) {
		dev_err(tof->dev,
			"LoadConfigurationFromFlash failed for 0x%06X (STATUS=0x%04X)\n",
			flash_addr, status);
		return -EIO;
	}

	/* Set the old mode again. */
	return tof_m2453_i2c_write_reg(tof, tof_regs(tof)->seq_mode, prev_mode);
}

int tof_m2453_find_usecase(struct tof_m2453 *tof, const char *name)
{
	unsigned int i;

	for (i = 0; i < tof->num_use_cases; i++) {
		if (!strcasecmp(tof->use_cases[i].name, name))
			return i;
	}
	return -ENOENT;
}

/*
 * ------------------------------------------------------------------ *
 * i2c driver template -- DISABLED
 *
 * This workspace integrates the engine above into the board's own driver
 * (irs2877a), so the standalone module registration is wrapped in #if 0.
 * Remove the #if 0 / #endif pair to build this directory as its own module
 * (kernel 6.6+ single-argument probe; 6.5 and older take (client, id)).
 *
 * Board glue must select the imager map before tof_m2453_probe_config():
 * either set tof->regs = &tof_regs_m2442 / &tof_regs_m2453 directly, or use
 * the "infineon,imager" device property handled below.
 * ------------------------------------------------------------------ *
 */
#if 0
static const struct of_device_id tof_m2453_of_match[] = {
	{ .compatible = "infineon,m2453" },
	{ }
};
MODULE_DEVICE_TABLE(of, tof_m2453_of_match);

static const struct i2c_device_id tof_m2453_i2c_id[] = {
	{ "tof_m2453", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tof_m2453_i2c_id);

static int tof_m2453_i2c_probe(struct i2c_client *client)
{
	struct tof_m2453 *tof;
	const char *imager = NULL;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	tof = devm_kzalloc(&client->dev, sizeof(*tof), GFP_KERNEL);
	if (!tof)
		return -ENOMEM;

	tof->dev = &client->dev;
	tof->client = client;
	tof->ops = dev_get_platdata(&client->dev);
	if (!tof->ops || !tof->ops->set_reset) {
		dev_err(&client->dev, "no board glue (reset callback) provided\n");
		return -ENODEV;
	}

	/*
	 * Pick the imager register map.  This MUST be right: the SPI bus
	 * master is at different addresses on the M2442 (IRS2877A) than on the
	 * M2453/M2455/M2457, so guessing wrong means writing reserved or
	 * analog registers.
	 */
	if (!device_property_read_string(&client->dev, "infineon,imager",
					 &imager) && imager) {
		tof->regs = tof_m2453_regs_by_name(imager);
		if (!tof->regs) {
			dev_err(&client->dev, "unknown infineon,imager = \"%s\"\n",
				imager);
			return -EINVAL;
		}
	}

	ret = tof_m2453_init_state(tof);
	if (ret)
		return ret;

	/* reset + SPI + small tables + the cached use case blocks */
	ret = tof_m2453_probe_config(tof);
	if (ret) {
		dev_err(&client->dev, "module configuration read failed: %d\n", ret);
		goto err_release;
	}

	ret = tof_m2453_bring_up(tof);
	if (ret) {
		dev_err(&client->dev, "imager bring-up failed: %d\n", ret);
		goto err_release;
	}

	i2c_set_clientdata(client, tof);

	/* TODO: register the V4L2 subdev, then expose the use cases as a menu
	 * control / VIDIOC_TOF_* ioctls whose setter calls
	 * tof_m2453_set_usecase(). */
	return 0;

err_release:
	tof_m2453_release(tof);
	return ret;
}

static void tof_m2453_i2c_remove(struct i2c_client *client)
{
	struct tof_m2453 *tof = i2c_get_clientdata(client);

	if (!tof)
		return;
	tof_m2453_stop_capture(tof);
	tof_m2453_release(tof);
}

static struct i2c_driver tof_m2453_i2c_driver = {
	.driver = {
		.name = "tof_m2453",
		.of_match_table = tof_m2453_of_match,
	},
	.probe = tof_m2453_i2c_probe,
	.remove = tof_m2453_i2c_remove,
	.id_table = tof_m2453_i2c_id,
};
module_i2c_driver(tof_m2453_i2c_driver);

MODULE_DESCRIPTION("Infineon ToF imager (M2453/M2455/M2457/M2442) register + module flash access");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("tof_viewer_opt");
#endif /* 0 */
