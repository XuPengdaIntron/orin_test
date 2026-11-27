/*
 * floodspot_mode_tbls.h - floodspot sensor mode tables
 *
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __FLOODSPOT_I2C_TABLES__
#define __FLOODSPOT_I2C_TABLES__

#include <media/camera_common.h>

#define FLOODSPOT_TABLE_WAIT_MS	0xff00
#define FLOODSPOT_TABLE_END	0xff01
#define FLOODSPOT_MAX_RETRIES	3
#define FLOODSPOT_WAIT_MS_STOP	1
#define FLOODSPOT_WAIT_MS_START	30
#define FLOODSPOT_WAIT_MS_STREAM	210
#define FLOODSPOT_GAIN_TABLE_SIZE 255

#define ovx3c_reg struct reg_8

enum {
	FLOODSPOT_MODE_Hybrid_Mode18_15fps,
	FLOODSPOT_MODE_Mode9_60Mhz_80Mhz_1000us_30fps,
};

static const int floodspot_15fps[] = {
	15,
};

static const int floodspot_30fps[] = {
	30,
};

static const int floodspot_60fps[] = {
	60,
};

static const int floodspot_120fps[] = {
	120,
};

static const struct camera_common_frmfmt floodspot_frmfmt[] = {
	{{240, 3258}, floodspot_15fps, 1, 0, FLOODSPOT_MODE_Hybrid_Mode18_15fps},
	{{240, 1629}, floodspot_30fps, 1, 0, FLOODSPOT_MODE_Mode9_60Mhz_80Mhz_1000us_30fps},
};
#endif /* __FLOODSPOT_I2C_TABLES__ */
