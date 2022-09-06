// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP
 */
#include <common.h>
#include <env.h>
#include <init.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/global_data.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <i2c.h>
#include <asm/io.h>
#include <power/pmic.h>
#include <power/bd71837.h>
#include "../common/tcpc.h"
#include <usb.h>
#include <imx_sip.h>
#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include <spi.h>
#include <spi_flash.h>
DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)
#define FPFA_POWER_GPIO IMX_GPIO_NR(3, 23)
#define PMIC_PWREN_GPIO IMX_GPIO_NR(4, 27)
#define PMIC_PWRWODOG_GPIO IMX_GPIO_NR(4, 25)
static iomux_v3_cfg_t const fpgapower_pads[] = {
	               IMX8MM_PAD_SAI5_RXD2_GPIO3_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_UART2_RXD_UART2_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART2_TXD_UART2_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
static iomux_v3_cfg_t const pmicgpio_pads[] = {
        IMX8MM_PAD_SAI2_MCLK_GPIO4_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL),
        IMX8MM_PAD_SAI2_TXC_GPIO4_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};


#ifdef CONFIG_NAND_MXS
#ifdef CONFIG_SPL_BUILD
#define NAND_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL2 | PAD_CTL_HYS)
#define NAND_PAD_READY0_CTRL (PAD_CTL_DSE6 | PAD_CTL_FSEL2 | PAD_CTL_PUE)
static iomux_v3_cfg_t const gpmi_pads[] = {
	IMX8MM_PAD_NAND_ALE_RAWNAND_ALE | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_CE0_B_RAWNAND_CE0_B | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_CE1_B_RAWNAND_CE1_B | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_CLE_RAWNAND_CLE | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA00_RAWNAND_DATA00 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA01_RAWNAND_DATA01 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA02_RAWNAND_DATA02 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA03_RAWNAND_DATA03 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA04_RAWNAND_DATA04 | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA05_RAWNAND_DATA05	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA06_RAWNAND_DATA06	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA07_RAWNAND_DATA07	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_RE_B_RAWNAND_RE_B | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_READY_B_RAWNAND_READY_B | MUX_PAD_CTRL(NAND_PAD_READY0_CTRL),
	IMX8MM_PAD_NAND_WE_B_RAWNAND_WE_B | MUX_PAD_CTRL(NAND_PAD_CTRL),
	IMX8MM_PAD_NAND_WP_B_RAWNAND_WP_B | MUX_PAD_CTRL(NAND_PAD_CTRL),
};
#endif

static void setup_gpmi_nand(void)
{
#ifdef CONFIG_SPL_BUILD
	imx_iomux_v3_setup_multiple_pads(gpmi_pads, ARRAY_SIZE(gpmi_pads));
#endif

	init_nand_clk();
}
#endif
struct myd_jx8mp_st{
	char pn[256];
	char sn[256];
	char mac0[256];
	char mac1[256];
	char reserve[1024];

};
#define FACTORY_DATA_OFFS 0x1f00000
static void factory_data_env_config(void)
{
	struct myd_jx8mp_st param;
	struct spi_flash *sf;
	int env_updated = 0;
	char *env;
	int ret;
	char mac0[18]={0};
	char mac1[18]={0};
	int i,j;


	/*
	 * Get values from factory-data area in SPI NOR
	 */
	sf = spi_flash_probe(CONFIG_SF_DEFAULT_BUS,
			     CONFIG_SF_DEFAULT_CS,
			     CONFIG_SF_DEFAULT_SPEED,
			     CONFIG_SF_DEFAULT_MODE);
	if (!sf) {
		printf("F-Data:Unable to access SPI NOR flash\n");
		goto err;
	}

	ret = spi_flash_read(sf, FACTORY_DATA_OFFS, sizeof(param),
			     (void *)&(param));
	if (ret) {
		printf("F-Data:Unable to read factory-data from SPI NOR\n");
		goto err;
	}
	printf("\n");

	printf(">>>PN=%s\n",param.pn);
	printf(">>>SN=%s\n",param.sn);
	if(is_valid_ethaddr(param.mac0)){
		memset(mac0,0,sizeof(mac0));
		snprintf(mac0,sizeof(mac0),"%02x:%02x:%02x:%02x:%02x:%02x",
			param.mac0[0],param.mac0[1],param.mac0[2],param.mac0[3],param.mac0[4],param.mac0[5]);

		
		printf(">>>MAC0=%s\n",mac0);

		env = env_get("ethaddr");
		if (strcmp(env, mac0)) {
			env_set("ethaddr",mac0);
			env_updated=1;
		}
		
	}
	if(is_valid_ethaddr(param.mac1)){
		memset(mac1,0,sizeof(mac1));
		snprintf(mac1,sizeof(mac1),"%02x:%02x:%02x:%02x:%02x:%02x",
			param.mac1[0],param.mac1[1],param.mac1[2],param.mac1[3],param.mac1[4],param.mac1[5]);

		
		printf(">>>MAC1=%s\n",mac1);

		env = env_get("eth1addr");
		if (strcmp(env, mac1)) {
			env_set("eth1addr",mac1);
			env_updated=1;
		}
	}
	/* Check if the environment was updated and needs to get stored */
	if (env_updated != 0) {
		printf("F-Data:Values don't match env values -> saving\n");
		env_save();
	} else {
		debug("F-Data:Values match current env values\n");
	}


	spi_flash_free(sf);

err:
	;
}
int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	init_uart_clk(1);

#ifdef CONFIG_NAND_MXS
	setup_gpmi_nand(); /* SPL will call the board_early_init_f */
#endif

	return 0;
}
static int pmic_i2c_reg_write(struct udevice *dev, uint addr, u8 data)
{
        //uint8_t valb;
        int err;
        err = dm_i2c_write(dev, addr, &data, 1);
        return err;
}

static void pmic_init_fpga(void)
{
        struct udevice *bus,*main_dev;
        int i2c_bus = 1;
        int ret=0;
        printf("init pmic for fpga\n");
        ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
        if (ret) {
                printf("%s: No bus %d\n", __func__, i2c_bus);
                return;
        }
        ret = dm_i2c_probe(bus, 0x4b, 0, &main_dev);
        if (ret) {
                printf("%s: Can't find device id=0x%x, on bus %d\n",__func__, 0x4b, i2c_bus);
                return;
        }
	pmic_i2c_reg_write(main_dev, BD718XX_PWRONCONFIG1, 0x0);
        pmic_i2c_reg_write(main_dev, BD718XX_REGLOCK, 0x01);
        pmic_i2c_reg_write(main_dev, BD718XX_BUCK1_VOLT_RUN, 0x1e);
        pmic_i2c_reg_write(main_dev, BD718XX_BUCK2_VOLT_RUN, 0x05);
        pmic_i2c_reg_write(main_dev, BD718XX_1ST_NODVS_BUCK_VOLT, 0x03);
        pmic_i2c_reg_write(main_dev, BD718XX_2ND_NODVS_BUCK_VOLT, 0x03);
        pmic_i2c_reg_write(main_dev, BD718XX_4TH_NODVS_BUCK_VOLT, 0x03);
        pmic_i2c_reg_write(main_dev, BD718XX_4TH_NODVS_BUCK_VOLT, 0x37);
        pmic_i2c_reg_write(main_dev, BD718XX_LDO1_VOLT, 0x06);
        pmic_i2c_reg_write(main_dev, BD718XX_LDO2_VOLT, 0x01);

        pmic_i2c_reg_write(main_dev, BD718XX_LDO3_VOLT, 0x0);
        pmic_i2c_reg_write(main_dev, BD718XX_LDO4_VOLT, 0x01);
        //pmic_i2c_reg_write(main_dev, BD71837_LDO5_VOLT, 0x0);
        pmic_i2c_reg_write(main_dev, BD718XX_LDO6_VOLT, 0x03);
        pmic_i2c_reg_write(main_dev, 0x21,0);
        pmic_i2c_reg_write(main_dev, BD718XX_REGLOCK, 0x11);
#if 1
        mdelay(20);
        imx_iomux_v3_setup_multiple_pads(
                pmicgpio_pads, ARRAY_SIZE(pmicgpio_pads));
        gpio_request(PMIC_PWREN_GPIO, "PMIC_EN");
        gpio_request(PMIC_PWRWODOG_GPIO, "PMIC_WDOG");
        gpio_direction_output(PMIC_PWREN_GPIO, 1);
        gpio_direction_output(PMIC_PWRWODOG_GPIO, 1);
#endif

        /* power on fpga */
        mdelay(20);
        imx_iomux_v3_setup_multiple_pads(
                fpgapower_pads, ARRAY_SIZE(fpgapower_pads));
        gpio_request(FPFA_POWER_GPIO, "FPGA_POWER_EN");
        gpio_direction_output(FPFA_POWER_GPIO, 1);

        return;
}

#if IS_ENABLED(CONFIG_FEC_MXC)

#define FEC_RST_PAD IMX_GPIO_NR(1, 5)
static iomux_v3_cfg_t const fec1_rst_pads[] = {
	        IMX8MM_PAD_GPIO1_IO05_GPIO1_IO5 | MUX_PAD_CTRL(NO_PAD_CTRL),
};
static void setup_iomux_fec(void)
{
	imx_iomux_v3_setup_multiple_pads(fec1_rst_pads,ARRAY_SIZE(fec1_rst_pads));
	gpio_request(FEC_RST_PAD, "fec1_rst");
	gpio_direction_output(FEC_RST_PAD, 1);
	mdelay(10);
	gpio_direction_output(FEC_RST_PAD, 0);
	mdelay(20);
	gpio_direction_output(FEC_RST_PAD, 1);
	mdelay(20);
}

 int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;
	setup_iomux_fec();
	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&gpr->gpr[1], 0x2000, 0);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

#ifndef CONFIG_DM_ETH
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x00);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x82ee);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);
#endif

	return 0;
}
#endif

#ifdef CONFIG_USB_TCPC
struct tcpc_port port1;
struct tcpc_port port2;

static int setup_pd_switch(uint8_t i2c_bus, uint8_t addr)
{
	struct udevice *bus;
	struct udevice *i2c_dev = NULL;
	int ret;
	uint8_t valb;

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: Can't find bus\n", __func__);
		return -EINVAL;
	}

	ret = dm_i2c_probe(bus, addr, 0, &i2c_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x\n",
			__func__, addr);
		return -ENODEV;
	}

	ret = dm_i2c_read(i2c_dev, 0xB, &valb, 1);
	if (ret) {
		printf("%s dm_i2c_read failed, err %d\n", __func__, ret);
		return -EIO;
	}
	valb |= 0x4; /* Set DB_EXIT to exit dead battery mode */
	ret = dm_i2c_write(i2c_dev, 0xB, (const uint8_t *)&valb, 1);
	if (ret) {
		printf("%s dm_i2c_write failed, err %d\n", __func__, ret);
		return -EIO;
	}

	/* Set OVP threshold to 23V */
	valb = 0x6;
	ret = dm_i2c_write(i2c_dev, 0x8, (const uint8_t *)&valb, 1);
	if (ret) {
		printf("%s dm_i2c_write failed, err %d\n", __func__, ret);
		return -EIO;
	}

	return 0;
}

int pd_switch_snk_enable(struct tcpc_port *port)
{
	if (port == &port1) {
		debug("Setup pd switch on port 1\n");
		return setup_pd_switch(1, 0x72);
	} else if (port == &port2) {
		debug("Setup pd switch on port 2\n");
		return setup_pd_switch(1, 0x73);
	} else
		return -EINVAL;
}

struct tcpc_port_config port1_config = {
	.i2c_bus = 1, /*i2c2*/
	.addr = 0x50,
	.port_type = TYPEC_PORT_UFP,
	.max_snk_mv = 5000,
	.max_snk_ma = 3000,
	.max_snk_mw = 40000,
	.op_snk_mv = 9000,
	.switch_setup_func = &pd_switch_snk_enable,
};

struct tcpc_port_config port2_config = {
	.i2c_bus = 1, /*i2c2*/
	.addr = 0x52,
	.port_type = TYPEC_PORT_UFP,
	.max_snk_mv = 9000,
	.max_snk_ma = 3000,
	.max_snk_mw = 40000,
	.op_snk_mv = 9000,
	.switch_setup_func = &pd_switch_snk_enable,
};

static int setup_typec(void)
{
	int ret;

	debug("tcpc_init port 2\n");
	ret = tcpc_init(&port2, port2_config, NULL);
	if (ret) {
		printf("%s: tcpc port2 init failed, err=%d\n",
		       __func__, ret);
	} else if (tcpc_pd_sink_check_charging(&port2)) {
		/* Disable PD for USB1, since USB2 has priority */
		port1_config.disable_pd = true;
		printf("Power supply on USB2\n");
	}

	debug("tcpc_init port 1\n");
	ret = tcpc_init(&port1, port1_config, NULL);
	if (ret) {
		printf("%s: tcpc port1 init failed, err=%d\n",
		       __func__, ret);
	} else {
		if (!port1_config.disable_pd)
			printf("Power supply on USB1\n");
		return ret;
	}

	return ret;
}

int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;
	struct tcpc_port *port_ptr;

	debug("board_usb_init %d, type %d\n", index, init);

	if (index == 0)
		port_ptr = &port1;
	else
		port_ptr = &port2;

	imx8m_usb_power(index, true);

	if (init == USB_INIT_HOST)
		tcpc_setup_dfp_mode(port_ptr);
	else
		tcpc_setup_ufp_mode(port_ptr);

	return ret;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;

	debug("board_usb_cleanup %d, type %d\n", index, init);

	if (init == USB_INIT_HOST) {
		if (index == 0)
			ret = tcpc_disable_src_vbus(&port1);
		else
			ret = tcpc_disable_src_vbus(&port2);
	}

	imx8m_usb_power(index, false);
	return ret;
}

int board_ehci_usb_phy_mode(struct udevice *dev)
{
	int ret = 0;
	enum typec_cc_polarity pol;
	enum typec_cc_state state;
	struct tcpc_port *port_ptr;

	if (dev_seq(dev) == 0)
		port_ptr = &port1;
	else
		port_ptr = &port2;

	tcpc_setup_ufp_mode(port_ptr);

	ret = tcpc_get_cc_status(port_ptr, &pol, &state);
	if (!ret) {
		if (state == TYPEC_STATE_SRC_RD_RA || state == TYPEC_STATE_SRC_RD)
			return USB_INIT_HOST;
	}

	return USB_INIT_DEVICE;
}

#endif

#define DISPMIX				9
#define MIPI				10

int board_init(void)
{
	pmic_init_fpga();
	struct arm_smccc_res res;

#ifdef CONFIG_USB_TCPC
	setup_typec();
#endif

	if (IS_ENABLED(CONFIG_FEC_MXC))
		setup_fec();

	arm_smccc_smc(IMX_SIP_GPC, IMX_SIP_GPC_PM_DOMAIN,
		      DISPMIX, true, 0, 0, 0, 0, &res);
	arm_smccc_smc(IMX_SIP_GPC, IMX_SIP_GPC_PM_DOMAIN,
		      MIPI, true, 0, 0, 0, 0, &res);

	return 0;
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "EVK");
	env_set("board_rev", "iMX8MM");
#endif
	factory_data_env_config();
	return 0;
}

#ifdef CONFIG_ANDROID_SUPPORT
bool is_power_key_pressed(void) {
	return (bool)(!!(readl(SNVS_HPSR) & (0x1 << 6)));
}
#endif

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /* TODO */
}
#endif /* CONFIG_ANDROID_RECOVERY */
#endif /* CONFIG_FSL_FASTBOOT */
