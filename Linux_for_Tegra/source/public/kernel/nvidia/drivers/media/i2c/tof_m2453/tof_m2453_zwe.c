// SPDX-License-Identifier: GPL-2.0
/*
 * tof_m2453_zwe.c -- minimal Zwetschge ("zwe") table parser.
 *
 * Port of the parts of tof_native/zwe_parser.py that the kernel actually
 * needs, i.e. only enough to find the use-case register blocks and the
 * bring-up register maps:
 *
 *   ToC at flash 0x2000   "ZWETSCHGE"  -> addresses/sizes of the sub tables
 *   "Elena"  table        -> per use case: name, GUID, geometry, fps and the
 *                            SequentialRegisterHeader (flash addr, size,
 *                            imager addr) of its dense register block
 *   "eLENA"  table        -> init / fw_page_1 / fw_page_2 / fw_start / start /
 *                            stop register lists (+ FW pages may instead be
 *                            sequential blocks pointed at by a header)
 *
 * Deliberately NOT ported (userspace keeps it, see README.md):
 *   - the calibration blob (211928 bytes here),
 *   - the frequency / exposure / raw-frame-set metadata: that is depth engine
 *     input, not driver input.
 *
 * Byte order: every table field is LITTLE endian, only the sequential
 * register blocks are BIG endian.
 */

#include <linux/crc32.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uuid.h>

#include <asm/unaligned.h>

#include "tof_m2453.h"

#define ZWE_TOC_MAGIC		"ZWETSCHGE"
#define ZWE_TOC_MAGIC_LEN	9
#define ZWE_TOUC_MAGIC		"Elena"
#define ZWE_TORM_MAGIC		"eLENA"
#define ZWE_MAGIC_LEN		5

struct tof_zwe_toc {
	u32 version;
	u32 crc_stored;
	u32 crc_calc;
	bool crc_ok;
	u32 register_maps_addr;
	u32 register_maps_size;
	u32 use_cases_addr;
	u32 use_cases_size;
	u32 calibration_addr;
	u32 calibration_size;
	u32 calibration_crc;
	u8 num_use_cases;
	char module_serial[TOF_MODULE_SERIAL_MAX];
	char module_name[TOF_MODULE_NAME_MAX];
};

/* ------------------------------------------------------------------ *
 * little endian readers (zwe_parser.py u8/u16/u24/u32)
 * ------------------------------------------------------------------ */
static u16 zwe_le16(const u8 *b, size_t off)
{
	return get_unaligned_le16(b + off);
}

static u32 zwe_le24(const u8 *b, size_t off)
{
	return (u32)b[off] | ((u32)b[off + 1] << 8) | ((u32)b[off + 2] << 16);
}

static u32 zwe_le32(const u8 *b, size_t off)
{
	return get_unaligned_le32(b + off);
}

/* zlib.crc32() equivalent: reflected, init all-ones, final xor. */
static u32 zwe_crc32(const u8 *data, size_t len)
{
	return crc32_le(~0u, data, len) ^ ~0u;
}

/* zwe_parser.py as_ascii(): stop at the first NUL, keep it printable. */
static void zwe_ascii(char *dst, size_t dst_size, const u8 *src, size_t src_len)
{
	size_t i;

	if (!dst_size)
		return;

	for (i = 0; i < src_len && i + 1 < dst_size; i++) {
		if (src[i] == '\0')
			break;
		dst[i] = (src[i] >= 0x20 && src[i] < 0x7F) ? (char)src[i] : '?';
	}
	dst[i] = '\0';
}

/* TOC_CRC_LEN_BY_VERSION (zwe_parser.py) */
static size_t zwe_toc_crc_len(u32 version)
{
	switch (version) {
	case 0x146:
		return 69;
	case 0x147:
	case 0x148:
	case 0x149:
		return 88;
	case 0x14A:
		return 110;
	case 0x14B:
		return 132;
	case 0x14C:
		return 137;
	default:
		return 0;
	}
}

/* ------------------------------------------------------------------ *
 * ToC
 * ------------------------------------------------------------------ */
static int zwe_read_toc(struct tof_m2453 *tof, struct tof_zwe_toc *toc)
{
	u8 *buf;
	size_t crc_len;
	int ret;

	buf = kmalloc(TOF_ZWE_TOC_READ_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = tof_m2453_flash_read(tof, TOF_ZWE_TOC_FLASH_ADDRESS, buf,
				   TOF_ZWE_TOC_READ_SIZE);
	if (ret)
	{
		dev_err(tof->dev, "tof_m2453_flash_read %d\n",
			ret);

		goto out;
	}

	if (memcmp(buf, ZWE_TOC_MAGIC, ZWE_TOC_MAGIC_LEN)) {
		dev_err(tof->dev, "no Zwetschge table of contents at flash 0x%X\n",
			TOF_ZWE_TOC_FLASH_ADDRESS);
		ret = -EINVAL;
		goto out;
	}

	toc->crc_stored = zwe_le32(buf, 9);
	toc->version = zwe_le24(buf, 13);

	crc_len = zwe_toc_crc_len(toc->version);
	if (!crc_len || crc_len > TOF_ZWE_TOC_READ_SIZE) {
		dev_err(tof->dev, "unsupported Zwetschge ToC version 0x%X\n",
			toc->version);
		ret = -EINVAL;
		goto out;
	}
	toc->crc_calc = zwe_crc32(buf + 13, crc_len - 13);
	toc->crc_ok = !tof->verify_crc || (toc->crc_stored == toc->crc_calc);

	toc->register_maps_addr = zwe_le24(buf, 46);
	toc->register_maps_size = zwe_le24(buf, 49);
	toc->calibration_addr = zwe_le24(buf, 52);
	toc->calibration_size = zwe_le24(buf, 55);
	toc->calibration_crc = zwe_le32(buf, 58);
	toc->num_use_cases = buf[62];
	toc->use_cases_addr = zwe_le24(buf, 63);
	toc->use_cases_size = zwe_le24(buf, 66);

	if (toc->version >= 0x147)
		zwe_ascii(toc->module_serial, sizeof(toc->module_serial),
			  buf + 69, 19);
	if (toc->version >= 0x14B)
		zwe_ascii(toc->module_name, sizeof(toc->module_name),
			  buf + 110, 16);

	if (!toc->crc_ok)
		dev_warn(tof->dev,
			 "Zwetschge ToC CRC mismatch (stored 0x%08X, calculated 0x%08X)\n",
			 toc->crc_stored, toc->crc_calc);
	ret = 0;
out:
	kfree(buf);
	return ret;
}

/* ------------------------------------------------------------------ *
 * "Elena" -- table of use cases
 * ------------------------------------------------------------------ */
static int zwe_parse_use_cases(struct tof_m2453 *tof, const u8 *table,
			       size_t size, u8 num_use_cases)
{
	bool new_style = tof->toc_version >= 0x148;
	bool rfs_eleven = tof->toc_version >= 0x149;
	unsigned int fixed = new_style ? 70 : 65;
	struct tof_usecase *list;
	size_t pos = 9;
	unsigned int i;
	int ret = 0;

	if (size < 9 || memcmp(table, ZWE_TOUC_MAGIC, ZWE_MAGIC_LEN)) {
		dev_err(tof->dev, "TableOfUseCases magic not found (expected 'Elena')\n");
		return -EINVAL;
	}

	list = kcalloc(num_use_cases, sizeof(*list), GFP_KERNEL);
	if (!list)
		return -ENOMEM;

	for (i = 0; i < num_use_cases; i++) {
		const u8 *b;
		struct tof_usecase *uc = &list[i];
		u16 block_size;
		u32 name_len, measure_count, frequency_count, reg_map_count;
		u32 stream_count, exposure_group_count, rfs_count, reserved_size;
		u32 base, expected;
		uuid_t uuid;

		if (pos + 2 > size) {
			dev_err(tof->dev, "ran out of data while reading use case %u\n", i);
			ret = -EINVAL;
			goto err;
		}

		/* the size field includes its own 2 bytes */
		block_size = zwe_le16(table, pos);
		if (block_size < 2 || pos + block_size > size) {
			dev_err(tof->dev, "use case %u: invalid block size 0x%X\n",
				i, block_size);
			ret = -EINVAL;
			goto err;
		}
		if (block_size < fixed) {
			dev_err(tof->dev,
				"use case %u: block of %u bytes is too small for ToC version 0x%X\n",
				i, block_size, tof->toc_version);
			ret = -EINVAL;
			goto err;
		}
		b = table + pos;

		/* SequentialRegisterHeader */
		uc->flash_addr = zwe_le24(b, 2);
		uc->flash_size = zwe_le24(b, 5);
		uc->imager_addr = zwe_le16(b, 8);

		uc->index = i;
		uc->width = zwe_le16(b, 10);
		uc->height = zwe_le16(b, 12);
		memcpy(uuid.b, b + 14, sizeof(uuid.b));
		snprintf(uc->guid, sizeof(uc->guid), "%pUb", &uuid);
		uc->fps_start = b[30];
		uc->fps_min = b[31];
		uc->fps_max = b[32];

		if (new_style) {
			name_len = b[58];
			measure_count = zwe_le16(b, 59);
			frequency_count = zwe_le16(b, 61);
			reg_map_count = zwe_le16(b, 63);
			stream_count = b[65];
			exposure_group_count = b[66];
			rfs_count = zwe_le16(b, 67);
			reserved_size = b[69];
		} else {
			name_len = b[53];
			measure_count = zwe_le16(b, 54);
			frequency_count = zwe_le16(b, 56);
			reg_map_count = zwe_le16(b, 58);
			stream_count = b[60];
			exposure_group_count = b[61];
			rfs_count = zwe_le16(b, 62);
			reserved_size = b[64];
		}

		/* zwe_parser.py recomputes the block size from the variable
		 * data and rejects a mismatch -- do the same, it is the only
		 * cheap way to be sure the version guess was right. */
		expected = fixed + name_len + 2 * measure_count +
			   4 * frequency_count + 6 * reg_map_count +
			   2 * stream_count + 6 * exposure_group_count +
			   (rfs_eleven ? 11u : 6u) * rfs_count + reserved_size;
		if (expected != block_size) {
			dev_err(tof->dev,
				"use case %u: block size %u does not match the variable data (expected %u)\n",
				i, block_size, expected);
			ret = -EINVAL;
			goto err;
		}

		base = fixed;
		zwe_ascii(uc->name, sizeof(uc->name), b + base, name_len);
		uc->raw_frames = measure_count ?
			(u8)zwe_le16(b, base + name_len) : 0;

		pos += block_size;
	}

	if (pos != size)
		dev_info(tof->dev, "%zu trailing byte(s) after the last use case\n",
			 size - pos);

	tof->use_cases = list;
	tof->num_use_cases = num_use_cases;
	return 0;

err:
	kfree(list);
	return ret;
}

/* ------------------------------------------------------------------ *
 * "eLENA" -- table of register maps
 * ------------------------------------------------------------------ */
static int zwe_parse_timed_list(struct tof_m2453 *tof, struct tof_reg_map *map,
				const u8 *table, size_t size, size_t *offset,
				u16 count)
{
	struct tof_reg_write *entries;
	unsigned int i;

	map->entries = NULL;
	map->count = 0;
	if (!count)
		return 0;

	if (*offset + (size_t)count * 6 > size) {
		dev_err(tof->dev, "register map overruns the table\n");
		return -EINVAL;
	}

	entries = kvmalloc_array(count, sizeof(*entries), GFP_KERNEL);
	if (!entries)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		const u8 *e = table + *offset + i * 6;

		entries[i].addr = zwe_le16(e, 0);
		entries[i].value = zwe_le16(e, 2);
		/* sleep times are stored in 32 us units */
		entries[i].delay_us =
			(u32)zwe_le16(e, 4) * TOF_ZWE_TIMED_LIST_SLEEP_UNIT;
	}

	map->entries = entries;
	map->count = count;
	*offset += (size_t)count * 6;
	return 0;
}

/*
 * A FW page may be stored as a dense sequential block in the flash instead of
 * an inline list (module_config.py: _sequential_registers + "block or inline").
 * header = (flash addr, size in bytes, imager addr).
 */
static int zwe_load_seq_block(struct tof_reg_map *map, struct tof_m2453 *tof,
			      u32 flash_addr, u32 size, u16 imager_addr)
{
	struct tof_reg_write *entries;
	u16 *values;
	unsigned int count, i;
	int ret;

	if (!size || !imager_addr)
		return 0;
	if (size % 2 || size > TOF_FW_BLOCK_MAX_BYTES) {
		dev_err(tof->dev, "sequential block of %u bytes is not usable\n", size);
		return -EINVAL;
	}

	count = size / 2;
	entries = kvmalloc_array(count, sizeof(*entries), GFP_KERNEL);
	if (!entries)
		return -ENOMEM;

	values = kvmalloc_array(count, sizeof(*values), GFP_KERNEL);
	if (!values) {
		kvfree(entries);
		return -ENOMEM;
	}

	ret = tof_m2453_flash_read_u16(tof, flash_addr, values, size);
	if (!ret) {
		for (i = 0; i < count; i++) {
			entries[i].addr = (u16)(imager_addr + i);
			entries[i].value = values[i];
			entries[i].delay_us = 0;
		}
		kvfree(map->entries);
		map->entries = entries;
		map->count = count;
	}

	kvfree(values);
	if (ret)
		kvfree(entries);
	return ret;
}

static int zwe_parse_register_maps(struct tof_m2453 *tof, u32 addr, u32 size)
{
	static const char * const names[6] = {
		"init", "fw_page_1", "fw_page_2", "fw_start", "start", "stop",
	};
	struct tof_reg_map *maps[6] = {
		&tof->maps.init, &tof->maps.fw_page_1, &tof->maps.fw_page_2,
		&tof->maps.fw_start, &tof->maps.start, &tof->maps.stop,
	};
	u16 counts[6];
	u32 fw_seq[2][3];
	u8 *table;
	size_t offset = 40;
	u32 total = 0;
	unsigned int i;
	int ret = 0;

	if (!addr || !size)
		return 0;
	if (size < 40)
		return -EINVAL;

	table = kmalloc(size, GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	ret = tof_m2453_flash_read(tof, addr, table, size);
	if (ret)
		goto out;

	if (memcmp(table, ZWE_TORM_MAGIC, ZWE_MAGIC_LEN)) {
		dev_err(tof->dev, "TableOfRegisterMaps magic not found (expected 'eLENA')\n");
		ret = -EINVAL;
		goto out;
	}

	if (tof->verify_crc) {
		u32 stored = zwe_le32(table, 5);
		u32 calc = zwe_crc32(table + 9, size - 9);

		if (stored != calc)
			dev_warn(tof->dev,
				 "TableOfRegisterMaps CRC mismatch (stored 0x%08X, calculated 0x%08X)\n",
				 stored, calc);
	}

	fw_seq[0][0] = zwe_le24(table, 12);
	fw_seq[0][1] = zwe_le24(table, 15);
	fw_seq[0][2] = zwe_le16(table, 18);
	fw_seq[1][0] = zwe_le24(table, 20);
	fw_seq[1][1] = zwe_le24(table, 23);
	fw_seq[1][2] = zwe_le16(table, 26);

	counts[0] = zwe_le16(table, 28);
	counts[1] = zwe_le16(table, 30);
	counts[2] = zwe_le16(table, 32);
	counts[3] = zwe_le16(table, 34);
	counts[4] = zwe_le16(table, 36);
	counts[5] = zwe_le16(table, 38);

	for (i = 0; i < 6; i++)
		total += counts[i];
	if (40 + (size_t)total * 6 > size) {
		dev_err(tof->dev, "register map table is truncated\n");
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < 6; i++) {
		ret = zwe_parse_timed_list(tof, maps[i], table, size, &offset,
					   counts[i]);
		if (ret) {
			dev_err(tof->dev, "cannot parse the %s register map\n", names[i]);
			goto out;
		}
	}

	/* FW pages: prefer the dense flash block when the header asks for one */
	for (i = 0; i < 2; i++) {
		if (!fw_seq[i][1])
			continue;
		if (maps[1 + i]->count)
			dev_info(tof->dev,
				 "fw_page_%u is both a flash block and an inline list; using the block\n",
				 i + 1);
		ret = zwe_load_seq_block(maps[1 + i], tof, fw_seq[i][0],
					 fw_seq[i][1], (u16)fw_seq[i][2]);
		if (ret)
			goto out;
	}

	dev_info(tof->dev,
		 "register maps: init=%u fw1=%u fw2=%u fwstart=%u start=%u stop=%u\n",
		 tof->maps.init.count, tof->maps.fw_page_1.count,
		 tof->maps.fw_page_2.count, tof->maps.fw_start.count,
		 tof->maps.start.count, tof->maps.stop.count);
out:
	kfree(table);
	return ret;
}

/* ------------------------------------------------------------------ *
 * entry point
 * ------------------------------------------------------------------ */
int tof_zwe_read_module_config(struct tof_m2453 *tof)
{
	struct tof_zwe_toc toc = { };
	u8 *table = NULL;
	int ret;

	ret = zwe_read_toc(tof, &toc);
	if (ret)
	{
		dev_err(tof->dev, "zwe_read_toc: %d\n",
			ret);

		return ret;
	}
		

	tof->toc_version = toc.version;
	tof->toc_crc_ok = toc.crc_ok;
	strscpy(tof->module_serial, toc.module_serial, sizeof(tof->module_serial));
	strscpy(tof->module_name, toc.module_name, sizeof(tof->module_name));

	dev_info(tof->dev,
		 "module %s (%s), zwe version 0x%X, ToC CRC %s\n",
		 tof->module_serial[0] ? tof->module_serial : "?",
		 tof->module_name[0] ? tof->module_name : "?",
		 toc.version, toc.crc_ok ? "ok" : "BAD");
	dev_info(tof->dev,
		 "calibration blob: flash 0x%X, %u bytes (left to userspace)\n",
		 toc.calibration_addr, toc.calibration_size);

	ret = zwe_parse_register_maps(tof, toc.register_maps_addr,
				      toc.register_maps_size);
	if (ret)
		return ret;

	if (!toc.num_use_cases || !toc.use_cases_size)
		return 0;

	table = kmalloc(toc.use_cases_size, GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	ret = tof_m2453_flash_read(tof, toc.use_cases_addr, table,
				   toc.use_cases_size);
	if (!ret)
		ret = zwe_parse_use_cases(tof, table, toc.use_cases_size,
					  toc.num_use_cases);
	kfree(table);
	return ret;
}
