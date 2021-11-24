// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018-2019 NXP
 */

#include <common.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <spl.h>
#include <asm/mach-imx/dma.h>
#include <power/pmic.h>
#include <power/bd71837.h>
#include "../common/tcpc.h"
#include <usb.h>
#include <sec_mipi_dsim.h>
#include <imx_mipi_dsi_bridge.h>
#include <mipi_dsi_panel.h>
#include <asm/mach-imx/video.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_UART2_RXD_UART2_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART2_TXD_UART2_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

#ifdef CONFIG_FSL_FSPI
int board_qspi_init(void)
{
	set_clk_qspi();

	return 0;
}
#endif


int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	init_uart_clk(1);



	return 0;
}

#ifdef CONFIG_FEC_MXC
#define FEC_RST_PAD IMX_GPIO_NR(4, 22)
static iomux_v3_cfg_t const fec1_rst_pads[] = {
	IMX8MM_PAD_SAI2_RXC_GPIO4_IO22 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_fec(void)
{
	imx_iomux_v3_setup_multiple_pads(fec1_rst_pads,
					 ARRAY_SIZE(fec1_rst_pads));

	gpio_request(FEC_RST_PAD, "fec1_rst");
	gpio_direction_output(FEC_RST_PAD, 0);
#if !defined(CONFIG_MOTORCOMM_YT)
	mdelay(15);
	gpio_direction_output(FEC_RST_PAD, 1);
	mdelay(100);
#endif
}

static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	setup_iomux_fec();

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&gpr->gpr[1],
			IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK, 0);
	return set_clk_enet(ENET_125MHZ);
}

int board_phy_config(struct phy_device *phydev)
{
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x00);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x82ee);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);
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

	if (dev->seq == 0)
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

int board_init(void)
{
#ifdef CONFIG_USB_TCPC
	setup_typec();
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

#ifdef CONFIG_FSL_FSPI
	board_qspi_init();
#endif

	return 0;
}

#ifdef CONFIG_VIDEO_MXS


#define _LVDS_Output_ 1
#ifdef _LVDS_Output_

// ????LVDS????CLK??
#define Panel_Pixel_CLK 5920    // 65MHz * 100

//--------------------------------------------//
// ????LVDS?????
#define _8_Bit_Color_           // 24 bit
//  #define _6_Bit_Color_ // 18 bit

// ????LVDS?????
#define _VESA_
//#define _JEIDA_

//#define _De_mode_
#define _Sync_Mode_

//--------------------------------------------//

#ifdef _VESA_
#define _VesaJeidaMode 0x00
#else
#define _VesaJeidaMode 0x20
#endif

#ifdef _De_mode_
#define _DE_Sync_mode 0x08
#else
#define _DE_Sync_mode 0x00
#endif

#ifdef _8_Bit_Color_
#define _ColorDeepth 0x13
#else
#define _ColorDeepth 0x17
#endif

//--------------------------------------------//

// ???????????????????? LVDS ????Timing:
static int LVDS_Panel_Timing[] =
//  H_act	 V_act	 H_total V_total H_BP	 H_sync  V_sync  V_BP
{ 1024, 600, 1344, 635, 140, 20, 3, 20 };       // 1024x600 Timing
//{ 1024, 768, 1344, 806, 160, 136, 6, 29 };  // 1024x768 Timing

union Temp
{
	u8	Temp8[4];
	u16 Temp16[2];
	u32 Temp32;
};

#endif




#define LT8912_MAIN 0x48
#define LT8912_SECOND 0x49
#define LT8912_THIRD 0x4a

enum
{
	H_act = 0,
	V_act,
	H_tol,
	V_tol,
	H_bp,
	H_sync,
	V_sync,
	V_bp
};

enum {
	 _32KHz = 0,
	 _44d1KHz,
	 _48KHz,

	 _88d2KHz,
	 _96KHz,
	 _176Khz,
	 _196KHz
};

u16 IIS_N[] = 
{
	4096, // 32K
	6272, // 44.1K
	6144, // 48K
	12544, // 88.2K
	12288, // 96K
	25088, // 176K
	24576 // 196K
};


u16 Sample_Freq[]=
{
	0x30, // 32K
	0x00, // 44.1K
	0x20, // 48K
	0x80, // 88.2K
	0xa0, // 96K
	0xc0, // 176K
	0xe0 //  196K
};

#define ADV7535_MAIN 0x3d
#define ADV7535_DSI_CEC 0x3c

static const struct sec_mipi_dsim_plat_data imx8mm_mipi_dsim_plat_data = {
	.version	= 0x1060200,
	.max_data_lanes = 4,
	.max_data_rate  = 1500000000ULL,
	.reg_base = MIPI_DSI_BASE_ADDR,
	.gpr_base = CSI_BASE_ADDR + 0x8000,
};

static int lt8912_i2c_reg_write(struct udevice *dev, uint addr, uint mask, uint data)
{
	uint8_t valb;
	int err;

	if (mask != 0xff) {
		err = dm_i2c_read(dev, addr, &valb, 1);
		if (err)
			return err;

		valb &= ~mask;
		valb |= data;
	} else {
		valb = data;
	}

	err = dm_i2c_write(dev, addr, &valb, 1);
	return err;
}

static int lt8912_i2c_reg_read(struct udevice *dev, uint8_t addr, uint8_t *data)
{
	uint8_t valb;
	int err;

	err = dm_i2c_read(dev, addr, &valb, 1);
	if (err)
		return err;

	*data = (int)valb;
	return 0;
}

static void lt8912_init(void)
{
	struct udevice *bus, *main_dev, *cec_dev,*third_dev;
	int i2c_bus = 1;
	int ret;
	unsigned int version[2];
	uint8_t val;
	static int MIPI_Timing[]={1920,1080,2200,1125,148,44,5,36};
	static int MIPI_Lane=4;
	unsigned char HDMI_VIC = 0x10;   // vic ,0x10: 1080P ;  0x04 : 720P ; Refer to the following list


#ifdef _LVDS_Output_
	
	union Temp	Core_PLL_Ratio;
	float       lvds_clock = 51.2; //MHZ
	float		f_DIV;
	float temp_float;
	u8     temp;
#endif



	gpio_request(IMX_GPIO_NR(1, 13), "LT8912 RESET");
	gpio_direction_output(IMX_GPIO_NR(1, 13), 0);
	mdelay(100);
	gpio_direction_output(IMX_GPIO_NR(1, 13), 1);


	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: No bus %d\n", __func__, i2c_bus);
		return;
	}

	ret = dm_i2c_probe(bus, LT8912_MAIN, 0, &main_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x, on bus %d\n",
			__func__, LT8912_MAIN, i2c_bus);
		return;
	}

	ret = dm_i2c_probe(bus, LT8912_SECOND, 0, &cec_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x, on bus %d\n",
			__func__, LT8912_SECOND, i2c_bus);
		return;
	}

	ret = dm_i2c_probe(bus, LT8912_THIRD, 0, &third_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x, on bus %d\n",
			__func__, LT8912_THIRD, i2c_bus);
		return;
	}

	lt8912_i2c_reg_read(main_dev, 0x00, &version[0]);
	lt8912_i2c_reg_read(main_dev, 0x01, &version[1]);

	printf("LT8912 ID: %02x, %02x\n",version[0], version[1]);


#ifdef _LVDS_Output_
	/* DigitalClockEn */
	lt8912_i2c_reg_write(main_dev, 0x08,0xff, 0xff);
	lt8912_i2c_reg_write(main_dev, 0x09,0xff, 0xff);
	lt8912_i2c_reg_write(main_dev, 0x0a,0xff, 0xff);
	lt8912_i2c_reg_write(main_dev, 0x0b,0xff, 0x7c);
	lt8912_i2c_reg_write(main_dev, 0x0c,0xff, 0xff);


	lt8912_i2c_reg_write(main_dev, 0x51,0xff, 0x15);
#else

	/* DigitalClockEn */
	lt8912_i2c_reg_write(main_dev, 0x08,0xff, 0xff);
	lt8912_i2c_reg_write(main_dev, 0x09,0xff, 0x81);
	lt8912_i2c_reg_write(main_dev, 0x0a,0xff, 0xff);
	lt8912_i2c_reg_write(main_dev, 0x0b,0xff, 0x64);
	lt8912_i2c_reg_write(main_dev, 0x0c,0xff, 0xff);

	lt8912_i2c_reg_write(main_dev, 0x44,0xff, 0x31);
	lt8912_i2c_reg_write(main_dev, 0x51,0xff, 0x1f);
#endif

	/* TxAnalog */
	lt8912_i2c_reg_write(main_dev, 0x31,0xff, 0xa1);
	lt8912_i2c_reg_write(main_dev, 0x32,0xff, 0xbf);
	lt8912_i2c_reg_write(main_dev, 0x33,0xff, 0x17);
	lt8912_i2c_reg_write(main_dev, 0x37,0xff, 0x00);
	lt8912_i2c_reg_write(main_dev, 0x38,0xff, 0x22);
	lt8912_i2c_reg_write(main_dev, 0x60,0xff, 0x82);

	lt8912_i2c_reg_write(main_dev, 0x3a,0xff, 0x00);

	/* CbusAnalog */
	lt8912_i2c_reg_write(main_dev, 0x39,0xff, 0x45);
	lt8912_i2c_reg_write(main_dev, 0x3b,0xff, 0x00);

	// MIPIAnalog()
	lt8912_i2c_reg_write(main_dev, 0x3e,0xff, 0xc6);
	lt8912_i2c_reg_write(main_dev, 0x41,0xff, 0x7c);

	/* HDMIPllAnalog */
	lt8912_i2c_reg_write(main_dev, 0x44,0xff, 0x31);
	lt8912_i2c_reg_write(main_dev, 0x55,0xff, 0x44);
	lt8912_i2c_reg_write(main_dev, 0x57,0xff, 0x01);
	lt8912_i2c_reg_write(main_dev, 0x5a,0xff, 0x02);



	/* MipiBasicSet */
	lt8912_i2c_reg_write(cec_dev, 0x10,0xff, 0x01);
	lt8912_i2c_reg_write(cec_dev, 0x11,0xff, 0x08);
	lt8912_i2c_reg_write(cec_dev, 0x12,0xff, 0x04);
	lt8912_i2c_reg_write(cec_dev, 0x13,0xff, MIPI_Lane %4); /*lanes %4 */
	lt8912_i2c_reg_write(cec_dev, 0x14,0xff, 0x00);

	lt8912_i2c_reg_write(cec_dev, 0x15,0xff, 0x00);
	lt8912_i2c_reg_write(cec_dev, 0x1a,0xff, 0x03);
	lt8912_i2c_reg_write(cec_dev, 0x1b,0xff, 0x03);


#ifdef _LVDS_Output_
	//	void LvdsPowerDown(void)
	lt8912_i2c_reg_write(main_dev, 0x44,0xff, 0x31);
#endif


	/* MIPIDig */
	lt8912_i2c_reg_write(cec_dev, 0x18,0xff, (u8)( MIPI_Timing[H_sync] % 256 )); /*hsync%256 */
	lt8912_i2c_reg_write(cec_dev, 0x19,0xff, (u8)( MIPI_Timing[V_sync] % 256 )); /* vsync%256*/
	lt8912_i2c_reg_write(cec_dev, 0x1c,0xff, (u8)( MIPI_Timing[H_act] % 256 )); /* hactive %256*/
	lt8912_i2c_reg_write(cec_dev, 0x1d,0xff, (u8)( MIPI_Timing[H_act] / 256 )); /* hactive/256 */

	lt8912_i2c_reg_write(cec_dev, 0x1e,0xff, 0x67);
	lt8912_i2c_reg_write(cec_dev, 0x2f,0xff, 0x0c);

	lt8912_i2c_reg_write(cec_dev, 0x34,0xff, (u8)( MIPI_Timing[H_tol] % 256 )); /* htotal%256 */
	lt8912_i2c_reg_write(cec_dev, 0x35,0xff, (u8)( MIPI_Timing[H_tol] / 256 )); /* htotal/256 */
	lt8912_i2c_reg_write(cec_dev, 0x36,0xff, (u8)( MIPI_Timing[V_tol] % 256 )); /* vtotal%256 */
	lt8912_i2c_reg_write(cec_dev, 0x37,0xff, (u8)( MIPI_Timing[V_tol] / 256 )); /* vtotal/256 */
	lt8912_i2c_reg_write(cec_dev, 0x38,0xff, (u8)( MIPI_Timing[V_bp] % 256 )); /* vbp%256 */
	lt8912_i2c_reg_write(cec_dev, 0x39,0xff, (u8)( MIPI_Timing[V_bp] / 256 )); /* vbp/256 */
	lt8912_i2c_reg_write(cec_dev, 0x3a,0xff, (u8)( ( MIPI_Timing[V_tol] - MIPI_Timing[V_act] - MIPI_Timing[V_bp] - MIPI_Timing[V_sync] ) % 256 )); /* vfp % 0x100 */
	lt8912_i2c_reg_write(cec_dev, 0x3b,0xff, (u8)( ( MIPI_Timing[V_tol] - MIPI_Timing[V_act] - MIPI_Timing[V_bp] - MIPI_Timing[V_sync] ) / 256)); /* vfp >> 8 */
	lt8912_i2c_reg_write(cec_dev, 0x3c,0xff, (u8)( MIPI_Timing[H_bp] % 256 )); /* hbp % 0x100 */
	lt8912_i2c_reg_write(cec_dev, 0x3d,0xff, (u8)( MIPI_Timing[H_bp] / 256 )); /* hbp >> 8 */
	lt8912_i2c_reg_write(cec_dev, 0x3e,0xff, (u8)( ( MIPI_Timing[H_tol] - MIPI_Timing[H_act] - MIPI_Timing[H_bp] - MIPI_Timing[H_sync] ) % 256 )); /* hfp % 0x100 */
	lt8912_i2c_reg_write(cec_dev, 0x3f,0xff, (u8)( ( MIPI_Timing[H_tol] - MIPI_Timing[H_act] - MIPI_Timing[H_bp] - MIPI_Timing[H_sync] ) / 256 )); /*  hfp >> 8 */


	/* DDSConfig */
	lt8912_i2c_reg_write(cec_dev, 0x4e,0xff, 0x52);
	lt8912_i2c_reg_write(cec_dev, 0x4f,0xff, 0xde);
	lt8912_i2c_reg_write(cec_dev, 0x50,0xff, 0xc0);
	lt8912_i2c_reg_write(cec_dev, 0x51,0xff, 0x80);
	lt8912_i2c_reg_write(cec_dev, 0x51,0xff, 0x00);

	lt8912_i2c_reg_write(cec_dev, 0x1f,0xff, 0x5e);
	lt8912_i2c_reg_write(cec_dev, 0x20,0xff, 0x01);
	lt8912_i2c_reg_write(cec_dev, 0x21,0xff, 0x2c);
	lt8912_i2c_reg_write(cec_dev, 0x22,0xff, 0x01);
	lt8912_i2c_reg_write(cec_dev, 0x23,0xff, 0xfa);
	lt8912_i2c_reg_write(cec_dev, 0x24,0xff, 0x00);
	lt8912_i2c_reg_write(cec_dev, 0x25,0xff, 0xc8);
	lt8912_i2c_reg_write(cec_dev, 0x26,0xff, 0x00);
	lt8912_i2c_reg_write(cec_dev, 0x27,0xff, 0x5e);
	lt8912_i2c_reg_write(cec_dev, 0x28,0xff, 0x01);
	lt8912_i2c_reg_write(cec_dev, 0x29,0xff, 0x2c);
	lt8912_i2c_reg_write(cec_dev, 0x2a,0xff, 0x01);
	lt8912_i2c_reg_write(cec_dev, 0x2b,0xff, 0xfa);
	lt8912_i2c_reg_write(cec_dev, 0x2c,0xff, 0x00);
	lt8912_i2c_reg_write(cec_dev, 0x2d,0xff, 0xc8);
	lt8912_i2c_reg_write(cec_dev, 0x2e,0xff, 0x00);

	lt8912_i2c_reg_write(main_dev, 0x03,0xff, 0x7f);
	mdelay(10);
	lt8912_i2c_reg_write(main_dev, 0x03,0xff, 0xff);

	lt8912_i2c_reg_write(cec_dev, 0x42,0xff, 0x64);
	lt8912_i2c_reg_write(cec_dev, 0x43,0xff, 0x00);
	lt8912_i2c_reg_write(cec_dev, 0x44,0xff, 0x04);
	lt8912_i2c_reg_write(cec_dev, 0x45,0xff, 0x00);
	lt8912_i2c_reg_write(cec_dev, 0x46,0xff, 0x59);
	lt8912_i2c_reg_write(cec_dev, 0x47,0xff, 0x00);
	lt8912_i2c_reg_write(cec_dev, 0x48,0xff, 0xf2);
	lt8912_i2c_reg_write(cec_dev, 0x49,0xff, 0x06);
	lt8912_i2c_reg_write(cec_dev, 0x4a,0xff, 0x00);
	lt8912_i2c_reg_write(cec_dev, 0x4b,0xff, 0x72);
	lt8912_i2c_reg_write(cec_dev, 0x4c,0xff, 0x45);
	lt8912_i2c_reg_write(cec_dev, 0x4d,0xff, 0x00);
	lt8912_i2c_reg_write(cec_dev, 0x52,0xff, 0x08);
	lt8912_i2c_reg_write(cec_dev, 0x53,0xff, 0x00);
	lt8912_i2c_reg_write(cec_dev, 0x54,0xff, 0xb2);
	lt8912_i2c_reg_write(cec_dev, 0x55,0xff, 0x00);
	lt8912_i2c_reg_write(cec_dev, 0x56,0xff, 0xe4);
	lt8912_i2c_reg_write(cec_dev, 0x57,0xff, 0x0d);
	lt8912_i2c_reg_write(cec_dev, 0x58,0xff, 0x00);
	lt8912_i2c_reg_write(cec_dev, 0x59,0xff, 0xe4);
	lt8912_i2c_reg_write(cec_dev, 0x5a,0xff, 0x8a);
	lt8912_i2c_reg_write(cec_dev, 0x5b,0xff, 0x00);
	lt8912_i2c_reg_write(cec_dev, 0x5c,0xff, 0x34);
	lt8912_i2c_reg_write(cec_dev, 0x1e,0xff, 0x4f);
	lt8912_i2c_reg_write(cec_dev, 0x51,0xff, 0x00);

	lt8912_i2c_reg_write(main_dev, 0xb2,0xff, 0x00); // 0x01:HDMI; 0x00: DVI

	/* AudioIIsEn */
	lt8912_i2c_reg_write(third_dev, 0x06,0xff, 0x08); // 0x09
	lt8912_i2c_reg_write(third_dev, 0x07,0xff, 0x00); // enable Audio: 0xF0;  Audio Mute: 0x00
	lt8912_i2c_reg_write(third_dev, 0x0f,0xff, 0x0b + Sample_Freq[_44d1KHz]);
	lt8912_i2c_reg_write(third_dev, 0x37,0xff, (u8)(IIS_N[_44d1KHz]/0x10000));
	lt8912_i2c_reg_write(third_dev, 0x36,0xff, (u8)((IIS_N[_44d1KHz]&0x00FFFF)/0x100));
	lt8912_i2c_reg_write(third_dev, 0x35,0xff, (u8)(IIS_N[_44d1KHz]&0x0000FF));
	lt8912_i2c_reg_write(third_dev, 0x34,0xff, 0xD2);

	lt8912_i2c_reg_write(third_dev, 0x3c,0xff, 0x41);

	/* AVI Packet Config*/
	lt8912_i2c_reg_write(third_dev, 0x3e,0xff, 0x0A);
	lt8912_i2c_reg_write(third_dev, 0x43,0xff, 0x46 - HDMI_VIC);
	lt8912_i2c_reg_write(third_dev, 0x44,0xff, 0x10);
	lt8912_i2c_reg_write(third_dev, 0x45,0xff, 0x19);
	lt8912_i2c_reg_write(third_dev, 0x47,0xff, 0x00 + HDMI_VIC);


	/* MIPIRxLogicRes */
	lt8912_i2c_reg_write(main_dev, 0x03,0xff, 0x7f);
	mdelay(10);
	lt8912_i2c_reg_write(main_dev, 0x03,0xff, 0xff);

	lt8912_i2c_reg_write(cec_dev, 0x51,0xff, 0x80);
	mdelay(10);
	lt8912_i2c_reg_write(cec_dev, 0x51,0xff, 0x00);


#ifdef _LVDS_Output_
//	Coll PLL?????????

	temp = (u8)(( lvds_clock*7 ) /25);

	lt8912_i2c_reg_write(main_dev, 0x50,0xff, 0x24);
	lt8912_i2c_reg_write(main_dev, 0x51,0xff, 0x05);
	lt8912_i2c_reg_write(main_dev, 0x52,0xff, 0x14);

	lt8912_i2c_reg_write(main_dev, 0x69, 0xff, temp );
	lt8912_i2c_reg_write(main_dev, 0x69, 0xff, 0x80 + temp );
	printf("LT8912 0x69 [%x] [%x]\n",temp,0x80 + temp);
	temp_float = lvds_clock*7/25;
	temp_float = (temp_float - (int)(temp_float))*16384;


//***********************************************************//
// ???????????????????Big-endian??
//	lt8912_i2c_reg_write(main_dev, 0x6c, 0xff, 0x80 + Core_PLL_Ratio.Temp8[2] );
//	lt8912_i2c_reg_write(main_dev, 0x6b, 0xff, Core_PLL_Ratio.Temp8[3] );


// ???????????????????Little-endian??
	temp = (u8)(temp_float/256+128);
	lt8912_i2c_reg_write(main_dev, 0x6c, 0xff, temp);
	printf("LT8912 0x6c [%x] \n",temp);
	temp = (u8)(temp_float-(int)(temp_float/256)*256);
	lt8912_i2c_reg_write(main_dev, 0x6b, 0xff, temp);
	printf("LT8912 6b[%x]\n",temp);
//***********************************************************//

	lt8912_i2c_reg_write( main_dev, 0x04, 0xff, 0xfb ); //core pll reset
	lt8912_i2c_reg_write( main_dev, 0x04, 0xff, 0xff );
//------------------------------------------//


	//  LVDS Output?????????
	//  void LVDS_Scale_Ratio(void)


	lt8912_i2c_reg_write( main_dev, 0x80, 0xff, 0x00 );
	lt8912_i2c_reg_write( main_dev, 0x81, 0xff, 0xff );
	lt8912_i2c_reg_write( main_dev, 0x82, 0xff, 0x03 );


	lt8912_i2c_reg_write( main_dev, 0x83, 0xff, (u8)( MIPI_Timing[H_act] % 256 ) );
	lt8912_i2c_reg_write( main_dev, 0x84, 0xff, (u8)( MIPI_Timing[H_act] / 256 ) );
	printf("LT8912 0x83 [%x] 0x84[%x]\n",(u8)( MIPI_Timing[H_act] % 256 ),(u8)( MIPI_Timing[H_act] / 256 ));

	lt8912_i2c_reg_write( main_dev, 0x85, 0xff, 0x80);
	lt8912_i2c_reg_write( main_dev, 0x86, 0xff, 0x10);





	lt8912_i2c_reg_write( main_dev, 0x87, 0xff, (u8)( LVDS_Panel_Timing[H_tol] % 256 ));
	lt8912_i2c_reg_write( main_dev, 0x88, 0xff, (u8)( LVDS_Panel_Timing[H_tol] / 256 ));
	lt8912_i2c_reg_write( main_dev, 0x89, 0xff, (u8)( LVDS_Panel_Timing[H_sync] % 256 ));
	lt8912_i2c_reg_write( main_dev, 0x8a, 0xff, (u8)( LVDS_Panel_Timing[H_bp] % 256 ));
	lt8912_i2c_reg_write( main_dev, 0x8b, 0xff, (u8)( ( LVDS_Panel_Timing[H_bp] / 256 ) * 0x80 + ( LVDS_Panel_Timing[V_sync] % 256 ) ));
	lt8912_i2c_reg_write( main_dev, 0x8c, 0xff, (u8)( LVDS_Panel_Timing[H_act] % 256 ));
	lt8912_i2c_reg_write( main_dev, 0x8d, 0xff, (u8)( LVDS_Panel_Timing[V_act] % 256 ));
	lt8912_i2c_reg_write( main_dev, 0x8e, 0xff, (u8)( ( LVDS_Panel_Timing[V_act] / 256 ) * 0x10 + ( LVDS_Panel_Timing[H_act] / 256 ) ));
	printf("LT8912 0x87 [%x] 0x88[%x]\n",(u8)( LVDS_Panel_Timing[H_tol] % 256 ), (u8)( LVDS_Panel_Timing[H_tol] / 256 ));
	printf("LT8912 0x89 [%x] 0x8a[%x]\n",(u8)( LVDS_Panel_Timing[H_sync] % 256 ),(u8)( LVDS_Panel_Timing[H_bp] % 256 ));
	printf("LT8912 0x8b [%x] 0x8c[%x]\n",(u8)( ( LVDS_Panel_Timing[H_bp] / 256 ) * 0x80 + ( LVDS_Panel_Timing[V_sync] % 256 ) ), (u8)( LVDS_Panel_Timing[H_act] % 256 ));
	printf("LT8912 0x8d [%x] 0x8e[%x]\n", (u8)( LVDS_Panel_Timing[V_act] % 256 ),(u8)( ( LVDS_Panel_Timing[V_act] / 256 ) * 0x10 + ( LVDS_Panel_Timing[H_act] / 256 ) ));

	temp_float				   = ( ( (float)( MIPI_Timing[H_act] - 1 ) ) / (float)( LVDS_Panel_Timing[H_act] - 1 ) ) * 4096;
	Core_PLL_Ratio.Temp32  = (u32)(temp_float+0.5);
	
	//***********************************************************//
	// ???????????????????Big-endian??

	//lt8912_i2c_reg_write( main_dev, 0x8f, 0xff, Core_PLL_Ratio.Temp8[3]);
	//lt8912_i2c_reg_write( main_dev, 0x90, 0xff, Core_PLL_Ratio.Temp8[2]);

	// ???????????????????Little-endian??
	temp = (u8)(Core_PLL_Ratio.Temp32&0xff);
	lt8912_i2c_reg_write( main_dev, 0x8f, 0xff, temp);
	printf("LT8912 0x8f [%x] \n",temp);
	temp = (u8)((Core_PLL_Ratio.Temp32>>8) & 0xff);
	lt8912_i2c_reg_write( main_dev, 0x90, 0xff, temp);
	printf("LT8912 0x90 [%x] \n",temp);
	//***********************************************************//

	temp_float				   = ( ( (float)( MIPI_Timing[V_act] - 1 ) ) / (float)( LVDS_Panel_Timing[V_act] - 1 ) ) * 4096;
	Core_PLL_Ratio.Temp32  = (u32)(temp_float+0.5);
	//***********************************************************//
	// ???????????????????Big-endian??

	//lt8912_i2c_reg_write( main_dev, 0x91, 0xff, Core_PLL_Ratio.Temp8[3]);
	//lt8912_i2c_reg_write( main_dev, 0x92, 0xff, Core_PLL_Ratio.Temp8[2]);

	// ???????????????????Little-endian??
	temp = (u8)(Core_PLL_Ratio.Temp32&0xff);
	lt8912_i2c_reg_write( main_dev, 0x91, 0xff, temp);
	printf("LT8912 0x91 [%x] \n",temp);
	temp = (u8)((Core_PLL_Ratio.Temp32>>8) & 0xff);
	lt8912_i2c_reg_write( main_dev, 0x92, 0xff, temp);
	printf("LT8912 0x92 [%x] \n",temp);
	//***********************************************************//

	lt8912_i2c_reg_write( main_dev, 0x7f, 0xff, 0x9c);


	lt8912_i2c_reg_write( main_dev, 0xa8, 0xff, _VesaJeidaMode + _DE_Sync_mode + _ColorDeepth);

	mdelay( 300 );

	//  void LvdsPowerUp(void)

	lt8912_i2c_reg_write(main_dev, 0x44, 0xff, 0x30);
#endif




}

#define DISPLAY_MIX_SFT_RSTN_CSR		0x00
#define DISPLAY_MIX_CLK_EN_CSR		0x04

   /* 'DISP_MIX_SFT_RSTN_CSR' bit fields */
#define BUS_RSTN_BLK_SYNC_SFT_EN	BIT(6)

   /* 'DISP_MIX_CLK_EN_CSR' bit fields */
#define LCDIF_PIXEL_CLK_SFT_EN		BIT(7)
#define LCDIF_APB_CLK_SFT_EN		BIT(6)

void disp_mix_bus_rstn_reset(ulong gpr_base, bool reset)
{
	if (!reset)
		/* release reset */
		setbits_le32(gpr_base + DISPLAY_MIX_SFT_RSTN_CSR, BUS_RSTN_BLK_SYNC_SFT_EN);
	else
		/* hold reset */
		clrbits_le32(gpr_base + DISPLAY_MIX_SFT_RSTN_CSR, BUS_RSTN_BLK_SYNC_SFT_EN);
}

void disp_mix_lcdif_clks_enable(ulong gpr_base, bool enable)
{
	if (enable)
		/* enable lcdif clks */
		setbits_le32(gpr_base + DISPLAY_MIX_CLK_EN_CSR, LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN);
	else
		/* disable lcdif clks */
		clrbits_le32(gpr_base + DISPLAY_MIX_CLK_EN_CSR, LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN);
}

struct mipi_dsi_client_dev lt8912_dev = {
	.channel	= 0,
	.lanes = 4,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE | 
			  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE,
	.name = "LT8912B",
};



#define FSL_SIP_GPC			0xC2000000
#define FSL_SIP_CONFIG_GPC_PM_DOMAIN	0x3
#define DISPMIX				9
#define MIPI				10

void do_enable_mipi2hdmi(struct display_info_t const *dev)
{


	/* LT8912 initialization */
	lt8912_init();

	/* enable the dispmix & mipi phy power domain */
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

	/* Put lcdif out of reset */
	disp_mix_bus_rstn_reset(imx8mm_mipi_dsim_plat_data.gpr_base, false);
	disp_mix_lcdif_clks_enable(imx8mm_mipi_dsim_plat_data.gpr_base, true);

	/* Setup mipi dsim */
	sec_mipi_dsim_setup(&imx8mm_mipi_dsim_plat_data);
	imx_mipi_dsi_bridge_attach(&lt8912_dev); /* attach adv7535 device */
}



struct display_info_t const displays[] = {{
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = NULL,
	.enable	= do_enable_mipi2hdmi,
	.mode	= {
		.name			= "MIPI2HDMI",
		.refresh		= 60,
		.xres			= 1920,
		.yres			= 1080,
		.pixclock		= 6734, /* 148500000 */
		.left_margin	= 148,
		.right_margin	= 88,
		.upper_margin	= 36,
		.lower_margin	= 4,
		.hsync_len		= 44,
		.vsync_len		= 5,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED

} }
};
size_t display_count = ARRAY_SIZE(displays);
#endif

int board_late_init(void)
{
#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /*TODO*/
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/
