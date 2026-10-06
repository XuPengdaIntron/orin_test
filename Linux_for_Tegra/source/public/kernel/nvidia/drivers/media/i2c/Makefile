subdir-ccflags-y += -Werror

ccflags-y += -I$(srctree)/drivers/media/platform/tegra
ccflags-y += -I$(srctree)/drivers/video/tegra/camera

obj-$(CONFIG_NV_VIDEO_IMX185) += nv_imx185.o
obj-$(CONFIG_NV_VIDEO_IMX185) += nv_imx185_v1.o
obj-$(CONFIG_NV_VIDEO_IMX477) += nv_imx477.o
obj-$(CONFIG_NV_VIDEO_IMX219) += nv_imx219.o
obj-$(CONFIG_NV_VIDEO_IMX268) += nv_imx268.o
obj-$(CONFIG_NV_VIDEO_IMX274) += nv_imx274.o
obj-$(CONFIG_NV_VIDEO_IMX318) += nv_imx318.o
obj-$(CONFIG_NV_VIDEO_LC898212) += nv_lc898212.o
obj-$(CONFIG_NV_VIDEO_OV5693) += nv_ov5693.o
obj-$(CONFIG_NV_VIDEO_OV9281) += nv_ov9281.o
obj-$(CONFIG_NV_VIDEO_OV10823) += nv_ov10823.o
obj-$(CONFIG_NV_VIDEO_OV23850) += nv_ov23850.o
obj-$(CONFIG_I2C_IOEXPANDER_PCA9570) += pca9570.o
obj-$(CONFIG_VIDEO_TC358840) += tc358840.o
obj-$(CONFIG_VIDEO_LT6911UXC) +=lt6911uxc.o
obj-$(CONFIG_I2C_IOEXPANDER_SER_MAX9295) += max9295.o
obj-$(CONFIG_I2C_IOEXPANDER_DESER_MAX9296) += max9296.o
obj-$(CONFIG_NV_VIDEO_IMX390) += nv_imx390.o
obj-$(CONFIG_NV_VIDEO_AR0234) += nv_ar0234.o
obj-$(CONFIG_NV_DESER_MAX96712) += max96712.o
obj-$(CONFIG_NV_VIDEO_IRS2877A) += nv_irs2877a.o
obj-$(CONFIG_NV_VIDEO_SZ220AT) += nv_sz220at.o
obj-$(CONFIG_NV_VIDEO_IRS2877A2) += nv_irs2877a2.o
nv_irs2877a2-y := nv_irs2877a_core.o tof_m2453/tof_m2453_i2c.o tof_m2453/tof_m2453_spi.o \
	tof_m2453/tof_m2453_zwe.o tof_m2453/tof_m2453_core.o  \
	tof_m2453/tof_m2453_regs.o
obj-$(CONFIG_NV_VIDEO_SZ220AT2) += nv_sz220at2.o
obj-$(CONFIG_NV_VIDEO_ISX031) += nv_isx031.o
