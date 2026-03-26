/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019-2025 NXP
 * Copyright 2024 Gilles Talis <gilles.talis@gmail.com>
 */

#ifndef __IMX8MP_NAVQP_H
#define __IMX8MP_NAVQP_H

#include <linux/sizes.h>
#include <linux/stringify.h>
#include <asm/arch/imx-regs.h>
#include <env/nxp/imx_env.h>

#if defined(CONFIG_CMD_NET)

#define CFG_FEC_MXC_PHYADDR          1

#define DWC_NET_PHYADDR			1

#endif

#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 1) \
	func(MMC, mmc, 2)

#include <config_distro_bootcmd.h>

/* Initial environment variables */
#define CFG_EXTRA_ENV_SETTINGS \
	BOOTENV

/* Link Definitions */

#define CFG_SYS_INIT_RAM_ADDR	0x40000000
#define CFG_SYS_INIT_RAM_SIZE	0x80000

/* 8GB DDR */
#define CFG_SYS_SDRAM_BASE		0x40000000
#define PHYS_SDRAM			0x40000000
#define PHYS_SDRAM_SIZE			0xC0000000	/* 3 GB */
#define PHYS_SDRAM_2			0x100000000
#define PHYS_SDRAM_2_SIZE		0x140000000	/* 5 GB */

#define CFG_MXC_UART_BASE		UART2_BASE_ADDR

#define NAVQP_BOOT_IMAGE_GUID \
        EFI_GUID(0x1ea7e557, 0xbb97, 0x496e, 0x98, 0xa6, \
                 0x00, 0x74, 0xcf, 0x7b, 0x55, 0xde)

#endif
