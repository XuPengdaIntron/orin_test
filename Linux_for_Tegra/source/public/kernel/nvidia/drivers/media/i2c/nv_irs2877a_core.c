/*
 * irs2877a.c - irs2877a sensor driver
 *
 * Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/max9295.h>

#include <media/tegracam_core.h>
#include "irs2877a2_mode_tbls.h"

#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/delay.h> 
#include "tof_m2453/tof_m2453.h"

#if 0
uint8_t irs2877a_init_setting[] = {

0x04,0x29,0x04,0x0B,0x00, 		 	////disable mipi output
0x04,0x29,0x00,0x06,0xF0, 				//// Turn off all links

0x04,0x29,0x00,0x10,0x11,  			////PHYA&PHYB select 3GBPS*/
0x04,0x29,0x00,0x11,0x11, 		 	////PHYC&PHYD select 3GBPS*/
0x00,0x32, 			////Delay 50ms
0x04,0x29,0x00,0x18,0x0F,  			////Oneshort reset
0x00,0x64, 			////delay 50-100ms

/*Video Pipe Selection*/
0x04,0x29,0x00,0xF0,0x62, 				////0b0110-0010  Pipe1=LINKB-PipeZ;Pipe0=LINKA-PipeZ;
0x04,0x29,0x00,0xF1,0xEA, 				////0b1110-1010  Pipe3=LINKD-PipeZ;Pipe2=LINKC-PipeZ; 
0x04,0x29,0x00,0xF4,0x0f, 				//// Turn on Pipe 0

/*------------------ MAX96717F Settings -----------------------*/
0x04,0x29,0x00,0x06,0xF4, 							////Enable LINKA
0x00,0x64, 														////Delay 50ms
//0x42 0x00 0x01 0x04 				//// 0x04:3G mode, 0x08:6G
//0x42 0x00 0x11 0x0F 				//// coax mode for link A. if STP, it's 0x0E.
//0x42 0x00 0x10 0x30				////RESET_ONESHOT
//usleep 50000 
//0x42 0x00 0x05 0xc0 				// enable lock
0x04, 0x42, 0x02, 0xcd, 0xf4, //1M,High prio,Jitter,output 0,GMSL2 tx
0x04, 0x42, 0x02, 0xce, 0x67, // pulldown,push-pull,id = 7
0x04, 0x42, 0x02, 0xcf, 0x07,// id = 7
0x04,0x42,0x03,0x30,0x40,				// enable mipi non-continuous clock, 1x4. One port with four data lanes.
0x04,0x42,0x03,0x31,0x10,		 	// Two data lanes
0x04,0x42,0x03,0x83,0x00,  						//// MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
//0x42 0x01 0x02 0x0E 		 	//// Disable heart beat and disable drift detect for pipe X
//0x42 0x01 0x0A 0x0E 		 	//// Disable heart beat and disable drift detect for pipe Y
//0x42 0x01 0x12 0x0E 				//// Disable heart beat and disable drift detect for pipe Z
//0x42 0x01 0x1A 0x0E 				//// Disable heart beat and disable drift detect for pipe U
0x04,0x42,0x03,0x08,0x64,				//CSI port selection for Video Pipeline Z
0x04,0x42,0x03,0x18,0x6c,				//Pipe_Z DT=0b0010-1100, bit6 is enable bit5:0 for datatype
0x04,0x42,0x03,0x11,0x40, 				//Start Video Pipe Z from CSI Port B
//0x42 0x03 0x15 0x80  					////  (independent_vs_mode): Enable
//0x42 0x03 0x0D 0x01  					//// FRONTTOP : FRONTTOP_5 | VC_SELZ_L (VC_SELZ_L): 0x1								 	 		 				    //// Double Mode Configuration
0x04,0x42,0x03,0x13,0x40,  					//// FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
0x04,0x42,0x03,0x1E,0x38,  					//// FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
0x04,0x42,0x00,0x02,0x43,				//Video transmit Channel Z enabled
0x04,0x42,0x00,0x01,0x24,
				  
0x04,0x42,0x02,0xbe,0x00,			////GPIO0 low(IMG_START)
0x04,0x42,0x02,0xc7,0x10,			////GPIO3 high(RESET_N)
0x00,0x1f,      
0x04,0x42,0x02,0xbe,0x10,			////GPIO0 high(IMG_START)          
0x04,0x42,0x02,0xc7,0x00,			////GPIO3 low(RESET_N)
0x00,0x1f,		  
0x04,0x42,0x00,0x01,0x04,
0x00,0x64,	
};
#endif

#if 0
uint8_t max96712_setting[] = {
        0x04,0x29,0x03,0x1c,0xeb,  // 1M,High prio,Jitter,output 0,GMSL2 tx
        0x04,0x29,0x03,0x1d,0xa7,  // pulldown,push-pull,id = 7
        0x04,0x29,0x03,0x1e,0x07,  // id = 7
        0x04,0x29,0x03,0x54,0xe7,  // linkB,High prio,Jitter,GMSL2 tx,id = 7
        0x04,0x29,0x03,0x55,0x07,  // linkB,disable GMSL2 rx, id = 7
        0x04,0x29,0x03,0x8a,0xe7, // linkC,High prio,Jitter,GMSL2 tx,id = 7
        0x04,0x29,0x03,0x8b,0x07, // linkC,disable GMSL2 rx, id = 7
        0x04,0x29,0x03,0xc1,0xe7, // linkD,High prio,Jitter,GMSL2 tx,id = 7
        0x04,0x29,0x03,0xc2,0x07,	//linkD,disable GMSL2 rx, id = 7
    0x04,0x42,0x00,0x01,0x24,
    //// video pipe 2  map FS/FE
    0x04,0x29,0x09,0x8B,0x07, 
    0x04,0x29,0x09,0xAD,0x15,   //// map to MIPI Controller 1
    0x04,0x29,0x09,0x8D,0x2c, 
    0x04,0x29,0x09,0x8E,0x2c,   //// map to VC0
    0x04,0x29,0x09,0x8F,0x00, 
    0x04,0x29,0x09,0x90,0x00, 
    0x04,0x29,0x09,0x91,0x01, 
    0x04,0x29,0x09,0x92,0x01, 

//// Double Mode Configuration
    0x04,0x29,0x09,0x73,0x01,  	////  (ALT_MEM_MAP12 CTRL1): Alternate memory map enabled

//// MIPI PHY Setting
//// Set Des in 2x4 mode
    0x04,0x29,0x08,0xA0,0x04, 
//// Set Lane Mapping for 4-lane port A
    0x04,0x29,0x08,0xA3,0xE4, 
    0x04,0x29,0x08,0xA4,0xE4, 

//// Set 4 lane D-PHY
    0x04,0x29,0x09,0x0A,0xC0, 
    0x04,0x29,0x09,0x4A,0xC0, 
    0x04,0x29,0x09,0x8A,0xC0, 
    0x04,0x29,0x09,0xCA,0xC0, 

//// Turn on MIPI PHYs
    0x04,0x29,0x08,0xA2,0x34, 

//// Hold DPLL in reset (config_soft_rst_n = 0) before changing the rate
    0x04,0x29,0x1C,0x00,0xF4, 
    0x04,0x29,0x1D,0x00,0xF4, 
    0x04,0x29,0x1E,0x00,0xF4, 
    0x04,0x29,0x1F,0x00,0xF4, 

//// Set Data rate to be 1200Mbps/lane
    0x04,0x29,0x04,0x15,0x2c, 	
    0x04,0x29,0x04,0x18,0x2c, 	
    0x04,0x29,0x04,0x1B,0x2c, 
    0x04,0x29,0x04,0x1E,0x2c, 
//// Release reset to DPLL (config_soft_rst_n = 1)
    0x04,0x29,0x1C,0x00,0xF5, 
    0x04,0x29,0x1D,0x00,0xF5, 
    0x04,0x29,0x1E,0x00,0xF5, 
    0x04,0x29,0x1F,0x00,0xF5, 
//// Enable LINKA-D
    0x04,0x29,0x00,0x06,0xF4, 	////Enable LINKA
    0x00,0x32,
};
#endif

uint8_t irs2877a_stream_on_setting[] = {
	0x04,0x6b,0x04,0x0B,0x42,	// PIPE X BPP = 0x10, enable CSI output
	0x04,0x6b,0x08,0xA0,0x84,       //force all mipi output
};

uint8_t irs2877a_stream_off_setting[] = {
	0x04,0x6b,0x04,0x0B,0x00,	// enable mipi output           
};

static const struct of_device_id irs2877a_of_match[] = {
	{ .compatible = "sensing,irs2877a2",},
	{ },
};
MODULE_DEVICE_TABLE(of, irs2877a_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_EXPOSURE_SHORT,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_HDR_EN,
};

struct irs2877a {
	struct i2c_client	*i2c_client;
	const struct i2c_device_id *id;
	struct v4l2_subdev	*subdev;
	struct device		*ser_dev;
	struct device		*dser_dev;
	struct gmsl_link_ctx	g_ctx;
	u32	frame_length;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
	struct tof_m2453 			tof;
};

#if 1
static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
};
#endif

static inline int irs2877a_read_reg(struct camera_common_data *s_data,
				u16 addr, u16 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFFFF;

	return err;
}

static int irs2877a_write_reg(struct camera_common_data *s_data,
				u16 addr, u16 val)
{
	int err;
	struct device *dev = s_data->dev;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(dev, "%s:i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}

int irs2877a_write_table(struct tegracam_device *tc_dev, uint8_t *pdata, int32_t setting_size)
{
	int err=0;
	struct device *dev = tc_dev->dev;
	struct irs2877a *priv = (struct irs2877a *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = priv->s_data;
	uint8_t i2c_slave;
	uint16_t reg_addr, reg_val, delay;
	u16  reg_val_read =0;
	int32_t i, len;

	for (i = 0; i < setting_size;) 
	{
		len = pdata[i];
		if (len == 5) 
		{
			i2c_slave = pdata[i + 1];
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			reg_val = (pdata[i + 4] << 8) | pdata[i + 5];
			
			err = irs2877a_write_reg(s_data, reg_addr, reg_val);
			if (err) {
				dev_err(dev,"%s: error\n", __func__);
				//return err;
			}	
			
			err = irs2877a_read_reg(s_data, reg_addr, &reg_val_read);
			if (err) {
				dev_err(dev,"%s: error\n", __func__);
				//return err;
			}
			else
			{
				if (reg_val_read != reg_val)
				{
					dev_err(dev,"%s: write error reg_addr=0x%x, 0x%x,0x%x\n", __func__, reg_addr, reg_val, reg_val_read);
				}
			}
			
			i = i + len + 1;
			
			//vin_info("write x8b %d@0x%02x: 0x%04x=0x%04x\n", bus, i2c_slave, reg_addr, reg_val);
		} else if (len == 0) 
		{
			delay = pdata[i + 1];
			mdelay(delay);
			i = i + 2;
		}
		else
		{
			dev_err(dev,"%s: wrong data\n", __func__);
			return -1;
		}
	}
	
	return 0;
}

static int call_shell_script(char *command)
{
    int result = 0;
    char cmdPath[] = "/bin/bash";
    char *cmdEnvp[] = {
        "HOME=/",
        "PATH=/sbin:/bin:/usr/sbin:/usr/bin",
        NULL,
    };
    char *cmdArgv[4] = {
        cmdPath,
        "-c",  
        NULL,
        NULL,
    };

    // 为保存完整命令分配足够的空间，这里假设命令不会太长，你可按需调整大小
    //char full_command[256];
    // 使用snprintf将命令内容复制到full_command中，格式化为bash -c "具体命令"的形式
    //snprintf(full_command, sizeof(full_command), "-c \"%s\"", command);
    cmdArgv[2] = command;
    
    result = call_usermodehelper(cmdPath, cmdArgv, cmdEnvp, UMH_WAIT_PROC);
    printk("The result of call_usermodehelper is %d\n", result);
    
    return result;
}


int32_t write_register(int32_t bus, uint8_t *pdata, int32_t setting_size)
{
	int32_t ret = 0;
	uint8_t i2c_slave;
	uint16_t reg_addr, value, delay;
	int32_t i, len, k;
	char cmd_buf[200];

	for (i = 0; i < setting_size;) 
	{
		len = pdata[i];
		if (len == 5) 
		{
			i2c_slave = pdata[i + 1];
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = (pdata[i + 4] << 8) | pdata[i + 5];
			snprintf(cmd_buf, sizeof(cmd_buf), "i2ctransfer -y %d w4@0x%x 0x%x 0x%x 0x%x 0x%x", bus, i2c_slave, pdata[i + 2], pdata[i + 3], pdata[i + 4], pdata[i + 5]);
			ret = call_shell_script(cmd_buf);
			k = 10;
			while (ret != 0 && k--) {
				printk("write sensor %d@0x%02x: 0x%04x=0x%02x ret %d retry %d\n", bus, i2c_slave, reg_addr, value, ret, k);
				mdelay(20);
				ret = call_shell_script(cmd_buf);
			}
			
			if (ret != 0) {
				printk("write x8b %d@0x%02x: 0x%04x=0x%04x error %d\n", bus, i2c_slave, reg_addr, value, ret);
				return ret;
			}
			
			i = i + len + 1;
			
			//vin_info("write x8b %d@0x%02x: 0x%04x=0x%04x\n", bus, i2c_slave, reg_addr, value);
		} else if (len == 4) 
		{
			i2c_slave = pdata[i + 1];
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			snprintf(cmd_buf, sizeof(cmd_buf), "i2ctransfer -y %d w3@0x%x 0x%x 0x%x 0x%x", bus, i2c_slave, pdata[i + 2], pdata[i + 3], pdata[i + 4]);
			ret = call_shell_script(cmd_buf);
			k = 10;
			while (ret != 0 && k--) 
			{
				printk("write serdes %d@0x%02x: 0x%04x=0x%02x ret %d retry %d\n", bus, i2c_slave, reg_addr, value, ret, k);
				mdelay(20);
				ret = call_shell_script(cmd_buf);
			}
			
			if (ret != 0) 
			{
				printk("write serdes %d@0x%02x: 0x%04x=0x%02x error %d\n", bus, i2c_slave, reg_addr, value, ret);
				return ret;
			}
			
			// usleep(100*1000);
			i = i + len + 1;
			//vin_info("write serdes %d@0x%02x: 0x%04x=0x%02x\n", bus, i2c_slave, reg_addr, value);
		} else if (len == 0) 
		{
			delay = pdata[i + 1];
			mdelay(delay);
			i = i + 2;
		}
	}
	
	return ret;
}




static struct mutex serdes_lock__;

static int irs2877a_gmsl_serdes_setup(struct irs2877a *priv)
{
	mutex_lock(&serdes_lock__);


	mutex_unlock(&serdes_lock__);
	return 0;
}

static void irs2877a_gmsl_serdes_reset(struct irs2877a *priv)
{
	mutex_lock(&serdes_lock__);


	mutex_unlock(&serdes_lock__);
}

static int irs2877a_power_on(struct camera_common_data *s_data)
{
	struct device *dev = s_data->dev;
	struct camera_common_power_rail *pw = s_data->power;
	int err=0;
	dev_info(dev, "irs2877a_power_on.\n");

	pw->state = SWITCH_ON;
	return err;
}

static int irs2877a_power_off(struct camera_common_data *s_data)
{
	struct device *dev = s_data->dev;
	struct camera_common_power_rail *pw = s_data->power;
	int err=0;
	dev_info(dev, "irs2877a_power_off.\n");
	pw->state = SWITCH_OFF;
	return err;
}

static int irs2877a_power_get(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	
	dev_info(tc_dev->dev, "irs2877a_power_get.\n");
	pw->state = SWITCH_OFF;
	return 0;
}

static int irs2877a_power_put(struct tegracam_device *tc_dev)
{
	dev_info(tc_dev->dev, "irs2877a_power_put.\n");
	return 0;
}

static int irs2877a_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	dev_info(tc_dev->dev, "irs2877a_set_group_hold.\n");
	return 0;
}

static int irs2877a_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	dev_info(tc_dev->dev, "irs2877a_set_gain.\n");
	return 0;
}

static int irs2877a_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct irs2877a *priv = (struct irs2877a *)tegracam_get_privdata(tc_dev);

	/* fixed 30fps */
	priv->frame_length = 30;
	return 0;
}

static int irs2877a_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	
	dev_info(tc_dev->dev, "irs2877a_set_exposure.\n");
	return 0;

}

static struct tegracam_ctrl_ops irs2877a_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = irs2877a_set_gain,
	.set_exposure = irs2877a_set_exposure,
	.set_exposure_short = irs2877a_set_exposure,
	.set_frame_rate = irs2877a_set_frame_rate,
	.set_group_hold = irs2877a_set_group_hold,
};

static struct camera_common_pdata *irs2877a_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *node = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!node)
		return NULL;

	match = of_match_device(irs2877a_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
		sizeof(*board_priv_pdata), GFP_KERNEL);

	err = of_property_read_string(node, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk not in DT\n");

	return board_priv_pdata;
}

#ifdef USE_SENSOR_INIT
int sensor_init(struct irs2877a *priv);
#endif

static int irs2877a_set_mode(struct tegracam_device *tc_dev)
{
	int err = 0;
	int usecase_idx = 0;
	struct device *dev = tc_dev->dev;
	struct irs2877a *priv = (struct irs2877a *)tegracam_get_privdata(tc_dev);
	struct tof_m2453 * tof = &priv->tof;
	struct camera_common_data *s_data = tc_dev->s_data;
	int mode_idx = s_data->mode_prop_idx;
	char * usecase_name = NULL;

	switch (mode_idx) {
	case 0:
		usecase_name = "Mode9_60Mhz_80Mhz_30fps";
		break;
	case 1:
		usecase_name = "Mode5_80Mhz_30fps";
		break;
	default:
		dev_err(dev, "Invalid mode index %d\n", mode_idx);
		return -EINVAL;
	}

	usecase_idx = tof_m2453_find_usecase(tof, usecase_name);
	if (usecase_idx < 0) {
		dev_err(dev, "Failed to find usecase\n");
		return -EINVAL;
	}

	dev_info(dev, "irs2877a_set_mode:mode_idx=%d, usecase_idx=%d.\n", mode_idx, usecase_idx);

	err = tof_m2453_set_usecase(tof, usecase_idx);
	if (err) 
	{
		dev_err(dev, "Failed to set usecase\n");
		return err;
	}

	#ifdef USE_SENSOR_INIT
	err = sensor_init(priv);
	if (err) 
	{
		dev_err(dev, "sensor_init:err=%d\n", err);
	}
	#endif

	return 0;
}

int sensor_start(struct tegracam_device *tc_dev) 
{
  int ret = 0;
  int setting_size = 0;
  int32_t bus = 2;
  struct device *dev = tc_dev->dev;


  
  uint8_t *pdata = irs2877a_stream_on_setting;
  setting_size = sizeof(irs2877a_stream_on_setting) / sizeof(uint8_t);
  ret = write_register(bus, pdata, setting_size);
  if (ret < 0) 
  {
    dev_err(dev, "irs2877a stream_on fail\n");
    return -1;
  }
  
  return ret;
}

int sensor_stop(struct tegracam_device *tc_dev) 
{
  struct device *dev = tc_dev->dev;
  int ret = 0;
  int setting_size = 0;
  int32_t bus = 2;

  uint8_t *pdata = irs2877a_stream_off_setting;
  setting_size = sizeof(irs2877a_stream_off_setting) / sizeof(uint8_t);
  
  ret = write_register(bus, pdata, setting_size);
  if (ret < 0) 
  {
    dev_err(dev, "irs2877a stream_off fail\n");
    return -1;
  }
   
  return ret;
}

static int irs2877a_start_streaming(struct tegracam_device *tc_dev)
{
	int err=0;
	struct device *dev = tc_dev->dev;
	struct irs2877a *priv = (struct irs2877a *)tegracam_get_privdata(tc_dev);
	//struct camera_common_data *s_data = priv->s_data;
	
	dev_info(dev, "irs2877a_start_streaming.\n");

	msleep(500);

	err = tof_m2453_start_capture(&priv->tof);
	if (err) 
	{
		dev_err(dev, "Failed to start capture\n");
		return err;
	}

	msleep(500);
	
	err = sensor_start(tc_dev);

	return err;
}

static int irs2877a_stop_streaming(struct tegracam_device *tc_dev)
{
	int err=0;
	struct irs2877a *priv = (struct irs2877a *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	dev_info(dev, "irs2877a_stop_streaming.\n");

	err = tof_m2453_stop_capture(&priv->tof);
	if (err) 
	{
		dev_err(dev, "Failed to stop capture\n");
	}
	
	err = sensor_stop(tc_dev);
	
	return err;
}

static struct camera_common_sensor_ops irs2877a_common_ops = {
	.numfrmfmts = ARRAY_SIZE(irs2877a_frmfmt),
	.frmfmt_table = irs2877a_frmfmt,
	.power_on = irs2877a_power_on,
	.power_off = irs2877a_power_off,
	.parse_dt = irs2877a_parse_dt,
	.power_get = irs2877a_power_get,
	.power_put = irs2877a_power_put,
	.set_mode = irs2877a_set_mode,
	.start_streaming = irs2877a_start_streaming,
	.stop_streaming = irs2877a_stop_streaming,
};

static int irs2877a_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops irs2877a_subdev_internal_ops = {
	.open = irs2877a_open,
};

static int irs2877a_board_setup(struct irs2877a *priv)
{
	struct tegracam_device *tc_dev = priv->tc_dev;
	struct device *dev = tc_dev->dev;
	struct device_node *node = dev->of_node;

	int err;

	err = of_property_read_u32(node, "reg", &priv->g_ctx.sdev_reg);
	if (err < 0) {
		dev_err(dev, "reg not found\n");
		goto error;
	}

	err = of_property_read_u32(node, "def-addr",
					&priv->g_ctx.sdev_def);
	if (err < 0) {
		dev_err(dev, "def-addr not found\n");
		goto error;
	}

	return 0;

error:
	return err;
}

#ifdef USE_SENSOR_INIT
int sensor_init(struct irs2877a *priv) 
{
  struct tegracam_device *tc_dev = priv->tc_dev;
  struct device *dev = tc_dev->dev;
  int ret = 0;
  int setting_size = 0;
  uint8_t *pdata = NULL;
  uint8_t try_count = 3;
  int32_t bus = 0;

  while(try_count)
  {
    pdata = irs2877a_init_setting;
    setting_size = sizeof(irs2877a_init_setting) / sizeof(uint8_t);
    dev_info(dev, "sensor_init enter setting_size %d \n", setting_size);
    ret =write_register(bus, pdata, setting_size);
    
    pdata = face_id_30fps_setting;
    setting_size = sizeof(face_id_30fps_setting) / sizeof(uint8_t);
    
    dev_info(dev, "sensor_init enter sensor_register_size %d\n", setting_size);
    ret = irs2877a_write_table(tc_dev, pdata, setting_size);
    if (ret < 0) 
    {
        dev_err(dev, "sensor_reg write register error\n");
        //return -HB_CAM_SERDES_CONFIG_FAIL;
    }

    pdata = max96712_setting;
    setting_size = sizeof(max96712_setting) / sizeof(uint8_t);
    dev_info(dev, "sensor_init enter max96712_register_size %d\n", setting_size);
    ret = write_register(bus, pdata, setting_size);
    if (ret < 0) 
    {
      dev_err(dev, "sensor_init write register error\n");
       --try_count;
      if(!try_count)
      {
        dev_err(dev, "sensor_init write register error, try 3 times!\n");
        return -1;
      } 
      else
      {
        continue;
      }
    }
    else
    {
      dev_info(dev, "sensor_init Done !\n");
      return 0;
    }
  }
  
  return 0;
}
#endif

static int irs2877a_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct tof_m2453 *tof;
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct tegracam_device *tc_dev;
	struct irs2877a *priv;
	int err;

	dev_info(dev, "probing v4l2 sensor.\n");
	
	printk("---irs2877a_probe 111---\r\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	priv = devm_kzalloc(dev, sizeof(struct irs2877a), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	tof = &priv->tof;
	tof->regs = tof_m2453_regs_by_name("irs2877a");
	tof->dev = dev;
	tof->client = client;
	// tof->ops = dev_get_platdata(&client->dev);
	// if (!tof->ops || !tof->ops->set_reset) {
	// 	dev_err(&client->dev, "no board glue (reset callback) provided\n");
	// 	return -ENODEV;
	// }

	err = tof_m2453_init_state(tof);
	if (err)
	{
		dev_err(&client->dev, "tof_m2453_init_state failed: %d\n", err);
		return err;
	}

	/* reset + SPI + small tables + the cached use case blocks */
	err = tof_m2453_probe_config(tof);
	if (err) 
	{
		dev_err(&client->dev, "module configuration read failed: %d\n", err);
		return EINVAL;
	}

	err = tof_m2453_bring_up(tof);
	if (err) 
	{
		dev_err(&client->dev, "imager bring-up failed: %d\n", err);
		return  err;
	}

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "irs2877a", sizeof(tc_dev->name));
	
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	
	tc_dev->sensor_ops = &irs2877a_common_ops;
	tc_dev->v4l2sd_internal_ops = &irs2877a_subdev_internal_ops;
	tc_dev->tcctrl_ops = &irs2877a_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}

	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;

	tegracam_set_privdata(tc_dev, (void *)priv);

	err = irs2877a_board_setup(priv);
	if (err) {
		tegracam_device_unregister(tc_dev);
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = irs2877a_gmsl_serdes_setup(priv);
	if (err) {
		dev_err(&client->dev,
			"%s gmsl serdes setup failed\n", __func__);
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_info(&client->dev, "Detected irs2877a sensor\n");

	return 0;
}

static int irs2877a_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct irs2877a *priv = (struct irs2877a *)s_data->priv;

	irs2877a_gmsl_serdes_reset(priv);

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct i2c_device_id irs2877a_id[] = {
	{ "irs2877a2", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, irs2877a_id);

static struct i2c_driver irs2877a_i2c_driver = {
	.driver = {
		.name = "irs2877a2",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(irs2877a_of_match),
	},
	.probe = irs2877a_probe,
	.remove = irs2877a_remove,
	.id_table = irs2877a_id,
};

static int __init irs2877a_init(void)
{
	mutex_init(&serdes_lock__);
	
	printk("---irs2877a_init---\r\n");

	return i2c_add_driver(&irs2877a_i2c_driver);
}

static void __exit irs2877a_exit(void)
{
	mutex_destroy(&serdes_lock__);
	
	printk("---irs2877a_exit---\r\n");

	i2c_del_driver(&irs2877a_i2c_driver);
}

module_init(irs2877a_init);
module_exit(irs2877a_exit);

MODULE_DESCRIPTION("Media Controller driver for Sensing irs2877a");
MODULE_AUTHOR("wangzhoujun@g-pulse.net");
MODULE_LICENSE("GPL v2");
