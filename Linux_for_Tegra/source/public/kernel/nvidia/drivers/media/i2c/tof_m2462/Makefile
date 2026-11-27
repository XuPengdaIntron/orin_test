# SPDX-License-Identifier: GPL-2.0
#
# Infineon M2462 (IRS2975C) dual-stream ToF imager: register + module flash
# access.
# Drop this directory into drivers/media/i2c/ (or drivers/misc/) of your
# kernel tree and add it to the parent Makefile/Kconfig, or point an external
# module build at it:
#
#   make -C <linux> M=$PWD/tof_m2462 modules
#
obj-$(CONFIG_TOF_M2462) += tof_m2462.o

tof_m2462-y := tof_m2462_core.o \
	       tof_m2462_i2c.o \
	       tof_m2462_regs.o \
	       tof_m2462_spi.o \
	       tof_m2462_zwe.o

ccflags-y += -Wall
