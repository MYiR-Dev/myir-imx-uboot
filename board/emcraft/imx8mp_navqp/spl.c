// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2018-2019, 2021 NXP
 * Copyright 2024 Gilles Talis <gilles.talis@gmail.com>
 */

#include <asm/arch/clock.h>
#include <asm/arch/ddr.h>
#include <asm/arch/imx8mp_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/global_data.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/sections.h>
#include <dm/device.h>
#include <dm/uclass.h>
#include <hang.h>
#include <init.h>
#include <log.h>
#include <power/pca9450.h>
#include <power/pmic.h>
#include <spl.h>

DECLARE_GLOBAL_DATA_PTR;

extern struct dram_timing_info dram_micron_8gb_timing;

int spl_board_boot_device(enum boot_device boot_dev_spl)
{
	return BOOT_DEVICE_BOOTROM;
}

#define DDR_BASE 0x40000000ULL
#define MIRROR   0xC0000000ULL

void spl_dram_init(void)
{
	int ret;
	volatile unsigned int *ptr;

	ret = ddr_init(&dram_micron_8gb_timing);

	if(ret == 0)
	{
		ptr = (volatile unsigned int *)DDR_BASE;
		ptr[0] = 0xCAFEBABE;

		ptr = (volatile unsigned int *)MIRROR;
		ptr[0] = 0xBEAFBEAF;

		invalidate_dcache_range((ulong)DDR_BASE,
					(ulong)DDR_BASE + 4);
		invalidate_dcache_range((ulong)MIRROR,
					(ulong)MIRROR + 4);

		ptr = (volatile unsigned int *)DDR_BASE;

		if (ptr[0] == 0xBEAFBEAF) {
			printf("4GB\n");
			printf ("Re-training for 4GByte Kingston memory\n");
			ddr_init(&dram_timing);
			/* Indicate 4GB chip to board_phys_sdram_size */
			ptr[0] = 0xBEAFBEAF;
		}
	} else {
		printf("8GB training failed\n");
		printf ("Re-training for 4GByte Kingston memory\n");
		ddr_init(&dram_timing);
		/* Indicate 4GB chip to board_phys_sdram_size */
		ptr = (volatile unsigned int *)DDR_BASE;
		ptr[0] = 0xBEAFBEAF;
	}
}

void spl_board_init(void)
{
	arch_misc_init();

	/*
	 * Set GIC clock to 500Mhz for OD VDD_SOC. Kernel driver does
	 * not allow to change it. Should set the clock after PMIC
	 * setting done. Default is 400Mhz (system_pll1_800m with div = 2)
	 * set by ROM for ND VDD_SOC
	 */
#if !defined(CONFIG_IMX8M_VDD_SOC_850MV)
	clock_enable(CCGR_GIC, 0);
	clock_set_target_val(GIC_CLK_ROOT, CLK_ROOT_ON | CLK_ROOT_SOURCE_SEL(5));
	clock_enable(CCGR_GIC, 1);

	puts("Normal Boot\n");
#endif
}

int power_init_board(void)
{
	struct udevice *dev;
	int ret;

	ret = pmic_get("pmic@25", &dev);
	if (ret == -ENODEV) {
		puts("Failed to get PMIC\n");
		return 0;
	}
	if (ret != 0)
		return ret;

	/* BUCKxOUT_DVS0/1 control BUCK123 output */
	pmic_reg_write(dev, PCA9450_BUCK123_DVS, 0x29);

	/*
	 * Increase VDD_SOC to typical value 0.95V before first
	 * DRAM access, set DVS1 to 0.85V for suspend.
	 * Enable DVS control through PMIC_STBY_REQ and
	 * set B1_ENMODE=1 (ON by PMIC_ON_REQ=H)
	 */
	if (CONFIG_IS_ENABLED(IMX8M_VDD_SOC_850MV))
		pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS0, 0x14);
	else
		pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS0, 0x1C);

	/* Set DVS1 to 0.85v for suspend. */
	pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS1, 0x14);

	/*
	 * Enable DVS control through PMIC_STBY_REQ and
	 * set B1_ENMODE=1 (ON by PMIC_ON_REQ=H).
	 */
	pmic_reg_write(dev, PCA9450_BUCK1CTRL, 0x59);

	/*
	 * Kernel uses OD/OD freq for SOC.
	 * To avoid timing risk from SOC to ARM,increase VDD_ARM to OD
	 * voltage 0.95V.
	 */
	pmic_reg_write(dev, PCA9450_BUCK2OUT_DVS0, 0x1C);

	return 0;
}

int board_fit_config_name_match(const char *name)
{
	if (is_imx8mp() &&
	    !strcmp(name, "imx8mp-navqp"))
		return 0;

	return -EINVAL;
}

void board_init_f(ulong dummy)
{
	struct udevice *dev;
	int ret;

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	arch_cpu_init();

	board_early_init_f();

	timer_init();

	ret = spl_early_init();
	if (ret) {
		debug("spl_early_init() failed: %d\n", ret);
		hang();
	}

	ret = uclass_get_device_by_name(UCLASS_CLK,
					"clock-controller@30380000",
					&dev);
	if (ret < 0) {
		printf("Failed to find clock node. Check device tree\n");
		hang();
	}

	preloader_console_init();

	enable_tzc380();

	power_init_board();

	/* DDR initialization */
	spl_dram_init();

	board_init_r(NULL, 0);
}
