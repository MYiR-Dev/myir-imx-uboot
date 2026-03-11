#
# Copyright 2022 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += myd_lmx9x.o

ifdef CONFIG_XPL_BUILD
obj-y += spl.o

obj-$(CONFIG_MYD_LMX9X_LPDDR4X_1G) += lpddr4x_timing_1g.o lpddr4x_timing_1866mts_1g.o
obj-$(CONFIG_MYD_LMX9X_LPDDR4X_2G) += lpddr4x_timing_2g.o lpddr4x_timing_1866mts_2g.o
endif
