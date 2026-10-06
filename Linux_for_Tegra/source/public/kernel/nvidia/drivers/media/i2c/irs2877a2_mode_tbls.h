/*
 * irs2877a2_mode_tbls.h - irs2877a2 sensor mode tables
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
#ifndef __IRS2877A2_I2C_TABLES__
#define __IRS2877A2_I2C_TABLES__

#include <media/camera_common.h>

#define IRS2877A_TABLE_WAIT_MS	0xff00
#define IRS2877A_TABLE_END	0xff01
#define IRS2877A_MAX_RETRIES	3
#define IRS2877A_WAIT_MS_STOP	1
#define IRS2877A_WAIT_MS_START	30
#define IRS2877A_WAIT_MS_STREAM	210
#define IRS2877A_GAIN_TABLE_SIZE 255

#define ovx3c_reg struct reg_8



enum {
	IRS2877A_MODE_640X2169_Mode9_60Mhz_80Mhz_30fps,
	IRS2877A_MODE_640X1205_Mode5_80Mhz_30fps,
	IRS2877A_MODE_START_STREAM,
	IRS2877A_MODE_STOP_STREAM,
};

static const int irs2877a_30fps[] = {
	30,
};

static const int irs2877a_60fps[] = {
	60,
};

static const int irs2877a_120fps[] = {
	120,
};

static const struct camera_common_frmfmt irs2877a_frmfmt[] = {
	{{640, 2169}, irs2877a_30fps, 1, 0, IRS2877A_MODE_640X2169_Mode9_60Mhz_80Mhz_30fps},
	{{640, 1205}, irs2877a_30fps, 1, 0, IRS2877A_MODE_640X1205_Mode5_80Mhz_30fps},
};
#endif /* __IRS2877A2_I2C_TABLES__ */
