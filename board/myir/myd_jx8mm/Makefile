#
# Copyright 2018 MYIR 
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += myd_jx8mm.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
ifdef CONFIG_IMX8M_4G_LPDDR4
obj-y += lpddr4_timing_4g.o
else
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing.o
obj-$(CONFIG_IMX8M_DDR4) += ddr4_timing.o
endif
endif
