#
# Copyright 2022 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += myir-lmx93x.o
obj-y += i2c_eeprom_myir.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
ifdef CONFIG_IMX9_LOW_DRIVE_MODE
obj-$(CONFIG_IMX93_EVK_LPDDR4X_1G) += lpddr4x_timing_ld.o
obj-$(CONFIG_IMX93_EVK_LPDDR4X_2G) += lpddr4x_timing_ld.o
else
ifdef CONFIG_IMX9_DRAM_INLINE_ECC
obj-$(CONFIG_IMX93_EVK_LPDDR4X_1G) += lpddr4x_timing_ecc.o
obj-$(CONFIG_IMX93_EVK_LPDDR4X_2G) += lpddr4x_timing_ecc.o
else
obj-$(CONFIG_IMX93_EVK_LPDDR4X_1G) += lpddr4x_timing_1g.o
obj-$(CONFIG_IMX93_EVK_LPDDR4X_2G) += lpddr4x_timing_2g.o
endif
endif
endif
