/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * Configuration settings for the Freescale i.MX6UL 14x14 EVK board.
 */
#ifndef __MYD_IMX6ULL_14X14_CONFIG_H
#define __MYD_IMX6ULL_14X14_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include <linux/stringify.h>
#include "mx6_common.h"
#include <asm/mach-imx/gpio.h>
#include <env/nxp/imx_env.h>

#define is_mx6ull_9x9_evk()	CONFIG_IS_ENABLED(TARGET_MX6ULL_9X9_EVK)

#ifdef CONFIG_TARGET_MX6ULL_9X9_EVK
#define BOOTARGS_CMA_SIZE   "cma=96M "
#else
#define BOOTARGS_CMA_SIZE   ""
#endif

#define BOOTARGS_EARLYCON "earlycon=uart8250,mmio32,0x02020000 "

#define CFG_MXC_UART_BASE		UART1_BASE

/* MMC Configs */
#ifdef CONFIG_FSL_USDHC
#define CFG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR

/* NAND pin conflicts with usdhc2 */
#ifdef CONFIG_NAND_MXS
#define CONFIG_SYS_FSL_USDHC_NUM	1
#else
#define CONFIG_SYS_FSL_USDHC_NUM	2
#endif
#endif

/*
 * Presence of NAND in the image (pins shared with USDHC2). Partition layout must
 * match Linux + CONFIG_MTDPARTS_DEFAULT when used; not tied to CONFIG_NAND_BOOT so
 * the same mtdparts describe hardware whenever the driver is built in.
 */
#ifdef CONFIG_NAND_MXS
#define MFG_NAND_PARTITION \
	"mtdparts=gpmi-nand:5m(boot),1m(env),10m(kernel),1m(dtb),-(rootfs)"
#else
#define MFG_NAND_PARTITION ""
#endif

/*
 * NAND boards only expose one USDHC instance (typically SD slot = mmc dev 0 in U-Boot);
 * MYD_EMMC_SKU uses mmc dev from CONFIG_SYS_MMC_ENV_DEV below.
 */
#ifdef CONFIG_NAND_MXS
#define MYD_MMCBOOT_LINE "mmcdev=0\0"
#define MYD_NAND_ENV_STRINGS                                                   \
	"nandargs=setenv bootargs console=${console},${baudrate} "                \
		BOOTARGS_EARLYCON                                                     \
		"ubi.mtd=rootfs root=ubi0:rootfs rootfstype=ubifs "                   \
		BOOTARGS_CMA_SIZE MFG_NAND_PARTITION "\0"                             \
	"nandboot=echo Booting from NAND ...; run nandargs; "                    \
		"mtd read kernel ${loadaddr} 0 0xa00000; "                            \
		"mtd read dtb ${fdt_addr} 0 0x100000; "                               \
		"if test ${tee} = yes; then "                                         \
			"nand read ${tee_addr} 0x6000000 0x400000; "                       \
			"bootm ${tee_addr} - ${fdt_addr}; "                               \
		"else bootz ${loadaddr} - ${fdt_addr}; fi\0"

#else /* !CONFIG_NAND_MXS */

#define MYD_MMCBOOT_LINE "mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV) "\0"
#define MYD_NAND_ENV_STRINGS ""

#endif /* CONFIG_NAND_MXS */

#define CFG_MFG_ENV_SETTINGS \
	CFG_MFG_ENV_SETTINGS_DEFAULT \
	"initrd_addr=0x86800000\0" \
	"initrd_high=0xffffffff\0" \
	"emmc_dev=1\0"\
	"emmc_ack=1\0"\
	"sd_dev=1\0" \
	"mtdparts=" MFG_NAND_PARTITION \
	"\0"\

/*
 * CFG_EXTRA_ENV_SETTINGS is shared for MMC/SD recovery and NAND field boot.
 * Default bootcmd is CONFIG_BOOTCOMMAND in defconfig (see myd_imx6ull_nand_ddr256_defconfig;
 * same MMC flow as emmc build, nandboot fallback instead of netboot alone):
 * MMC first (FAT partition ${mmcpart}), then nandboot fallback when NAND is compiled in.
 *
 * Keeping CONFIG_NAND_BOOT=y does not change this anymore; only use NAND_BOOT where
 * the SoC/boot ROM NAND path truly requires it at link time / SPL build.
 */
/* Match CONFIG_DEFAULT_DEVICE_TREE from the active defconfig (nand-ddr256 / nand-ddr512 / emmc). */
#ifdef CONFIG_DEFAULT_DEVICE_TREE
#define MYD_FDT_FILE_ENV "fdt_file=" CONFIG_DEFAULT_DEVICE_TREE ".dtb\0"
#else
#define MYD_FDT_FILE_ENV "fdt_file=myd-y6ull-14x14.dtb\0"
#endif

#define CFG_EXTRA_ENV_SETTINGS \
	CFG_MFG_ENV_SETTINGS \
	TEE_ENV \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"console=ttymxc0\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	MYD_FDT_FILE_ENV \
	"fdt_addr=0x83000000\0" \
	"tee_addr=0x84000000\0" \
	"tee_file=undefined\0" \
	"boot_fdt=try\0" \
	"fit_config=\0" \
	"boot_fit=try\0" \
	"ip_dyn=yes\0" \
	"panel=MYIR-LCD-7-800x480\0" \
	"splashimage=0x8c000000\0" \
	"splashpos=m,m\0" \
	MYD_MMCBOOT_LINE \
	"mmcpart=1\0" \
	"bootslot=dualA\0" \
	"mmcroot_a=/dev/mmcblk1p2 rootwait rw\0" \
	"mmcroot_b=/dev/mmcblk1p4 rootwait rw\0" \
	"mmcroot=/dev/mmcblk0p2 rootwait rw\0" \
	"mmcautodetect=yes\0" \
		"loglevel=7\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " BOOTARGS_EARLYCON \
		BOOTARGS_CMA_SIZE \
		"root=${mmcroot} bootslot=${bootslot} loglevel=${loglevel}\0" \
	"loadbootscript=" \
		"fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfitimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} fitImage-signed\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"loadtee=fatload mmc ${mmcdev}:${mmcpart} ${tee_addr} ${tee_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run select_bootslot; " \
		"run mmcargs; " \
		"if test ${tee} = yes; then " \
			"run loadfdt; run loadtee; bootm ${tee_addr} - ${fdt_addr}; " \
		"else " \
			"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
				"if run loadfdt; then " \
					"bootz ${loadaddr} - ${fdt_addr}; " \
				"else " \
					"if test ${boot_fdt} = try; then " \
						"bootz; " \
					"else " \
						"echo WARN: Cannot load the DT; " \
					"fi; " \
				"fi; " \
			"else " \
				"bootz; " \
			"fi; " \
		"fi;\0" \
	"mmcfboot=echo Booting fitImage from mmc ...; " \
		"setenv image fitImage; " \
		"mmc dev ${mmcdev}; " \
		"if mmc rescan; then " \
			"if run loadimage; then " \
				"run select_bootslot; " \
				"run mmcargs; " \
				"if test -n ${fit_config}; then " \
					"bootm ${loadaddr}#${fit_config}; " \
				"else " \
					"bootm ${loadaddr}; " \
				"fi; " \
			"fi; " \
		"fi;\0" \
	"mmchabboot=echo Booting zImage with HAB+TEE ...; " \
		"if hab_auth_img ${loadaddr} ${filesize} 0x800000; then " \
			"run loadfdt; " \
			"run loadtee; " \
			"run select_bootslot; " \
			"run mmcargs; " \
			"bootm ${tee_addr} - ${fdt_addr}; " \
		"else " \
			"echo HAB authentication FAILED for kernel at ${loadaddr}; " \
		"fi;\0" \
	"mmchabfitboot=echo Booting fitImage with HAB+TEE ...; " \
		"run select_bootslot; " \
		"run mmcargs; " \
		"bootm ${loadaddr};\0" \
	"select_bootslot=" \
		"if test ${mmcdev} = 1; then " \
			"if test ${bootslot} = dualA; then " \
				"setenv mmcroot ${mmcroot_a}; " \
			"else " \
				"setenv mmcroot ${mmcroot_b}; " \
			"fi; " \
		"fi;\0" \
	"switch_bootslot=" \
		"if test ${mmcdev} = 1; then " \
			"if test ${bootslot} = dualA; then " \
				"setenv bootslot dualB; " \
				"setenv mmcroot ${mmcroot_b}; " \
			"else " \
				"setenv bootslot dualA; " \
				"setenv mmcroot ${mmcroot_a}; " \
			"fi; saveenv; " \
		"fi;\0" \
	"altbootcmd=run switch_bootslot; run bootcmd\0" \
	MYD_NAND_ENV_STRINGS \
	"netargs=setenv bootargs console=${console},${baudrate} " BOOTARGS_EARLYCON \
		BOOTARGS_CMA_SIZE \
		"root=/dev/nfs " \
	"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
		"netboot=echo Booting from net ...; " \
		"${usb_net_cmd}; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"if test ${tee} = yes; then " \
			"${get_cmd} ${tee_addr} ${tee_file}; " \
			"${get_cmd} ${fdt_addr} ${fdt_file}; " \
			"bootm ${tee_addr} - ${fdt_addr}; " \
		"else " \
			"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
				"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
					"bootz ${loadaddr} - ${fdt_addr}; " \
				"else " \
					"if test ${boot_fdt} = try; then " \
						"bootz; " \
					"else " \
						"echo WARN: Cannot load the DT; " \
					"fi; " \
				"fi; " \
			"else " \
				"bootz; " \
			"fi; " \
		"fi;\0" \
		"findfdt="\
			"if test $fdt_file = undefined; then " \
				"if test $board_name = ULZ-EVK && test $board_rev = 14X14; then " \
					"setenv fdt_file imx6ulz-14x14-evk.dtb; fi; " \
				"if test $board_name = EVK && test $board_rev = 9X9; then " \
					"setenv fdt_file imx6ull-9x9-evk.dtb; fi; " \
				"if test $board_name = EVK && test $board_rev = 14X14; then " \
					"setenv fdt_file imx6ull-14x14-evk.dtb; fi; " \
				"if test $fdt_file = undefined; then " \
					"echo WARNING: Could not determine dtb to use; " \
				"fi; " \
			"fi;\0" \
		"findtee="\
			"if test $tee_file = undefined; then " \
				"if test $board_name = ULZ-EVK && test $board_rev = 14X14; then " \
					"setenv tee_file uTee-6ulzevk; fi; " \
				"if test $board_name = EVK && test $board_rev = 9X9; then " \
					"setenv tee_file uTee-6ullevk; fi; " \
				"if test $board_name = EVK && test $board_rev = 14X14; then " \
					"setenv tee_file uTee-6ullevk; fi; " \
				"if test $tee_file = undefined; then " \
					"echo WARNING: Could not determine tee to use; " \
				"fi; " \
			"fi;\0" \

/* Miscellaneous configurable options */

/* Physical Memory Map */
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CFG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CFG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CFG_SYS_INIT_RAM_SIZE	IRAM_SIZE

/* environment organization */

/* NAND stuff */
#define CFG_SYS_NAND_BASE		0x40000000
#endif
