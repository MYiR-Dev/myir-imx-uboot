// SPDX-License-Identifier: GPL-2.0+
/*
 * motor PHY drivers
 *
 * Copyright 2011, 2013 Freescale Semiconductor, Inc.
 * author Coin.du
 */
#include <common.h>
#include <phy.h>

#define MOTORCOMM_PHY_ID_MASK	0x00000fff
#define MOTORCOMM_MPHY_ID_MASK	0x0000ffff

#define PHY_ID_YT8010		0x00000309
#define PHY_ID_YT8510		0x00000109
#define PHY_ID_YT8511		0x0000010a
#define PHY_ID_YT8512		0x00000118
#define PHY_ID_YT8512B		0x00000128
#define PHY_ID_YT8521		0x0000011a
#define PHY_ID_YT8531S		0x0000091a
#define PHY_ID_YT8531		0x0000091b
//#define PHY_ID_YT8614		0x0000e899


#define REG_PHY_SPEC_STATUS		0x11
#define REG_DEBUG_ADDR_OFFSET		0x1e
#define REG_DEBUG_DATA			0x1f

#define YT8512_EXTREG_AFE_PLL		0x50
#define YT8512_EXTREG_EXTEND_COMBO	0x4000
#define YT8512_EXTREG_LED0		0x40c0
#define YT8512_EXTREG_LED1		0x40c3

#define YT8512_EXTREG_SLEEP_CONTROL1	0x2027

#define YT_SOFTWARE_RESET		0x8000

#define YT8512_CONFIG_PLL_REFCLK_SEL_EN	0x0040
#define YT8512_CONTROL1_RMII_EN		0x0001
#define YT8512_LED0_ACT_BLK_IND		0x1000
#define YT8512_LED0_DIS_LED_AN_TRY	0x0001
#define YT8512_LED0_BT_BLK_EN		0x0002
#define YT8512_LED0_HT_BLK_EN		0x0004
#define YT8512_LED0_COL_BLK_EN		0x0008
#define YT8512_LED0_BT_ON_EN		0x0010
#define YT8512_LED1_BT_ON_EN		0x0010
#define YT8512_LED1_TXACT_BLK_EN	0x0100
#define YT8512_LED1_RXACT_BLK_EN	0x0200
#define YT8512_SPEED_MODE		0xc000
#define YT8512_DUPLEX			0x2000

#define YT8512_SPEED_MODE_BIT		14
#define YT8512_DUPLEX_BIT		13
#define YT8512_EN_SLEEP_SW_BIT		15

#define YT8521_EXTREG_SLEEP_CONTROL1	0x27
#define YT8521_EN_SLEEP_SW_BIT		15

#define YT8521_SPEED_MODE		0xc000
#define YT8521_DUPLEX			0x2000
#define YT8521_SPEED_MODE_BIT		14
#define YT8521_DUPLEX_BIT		13
#define YT8521_LINK_STATUS_BIT		10

/* based on yt8521 wol config register */
#define YTPHY_UTP_INTR_REG             0x12
/* WOL Event Interrupt Enable */
#define YTPHY_WOL_INTR            BIT(6)

/* Magic Packet MAC address registers */
#define YTPHY_MAGIC_PACKET_MAC_ADDR2                 0xa007
#define YTPHY_MAGIC_PACKET_MAC_ADDR1                 0xa008
#define YTPHY_MAGIC_PACKET_MAC_ADDR0                 0xa009

#define YTPHY_WOL_CFG_REG		0xa00a
#define YTPHY_WOL_CFG_TYPE		BIT(0)	/* WOL TYPE */
#define YTPHY_WOL_CFG_EN		BIT(3)	/* WOL Enable */
#define YTPHY_WOL_CFG_INTR_SEL	BIT(6)	/* WOL Event Interrupt Enable */
#define YTPHY_WOL_CFG_WIDTH1	BIT(1)	/* WOL Pulse Width */
#define YTPHY_WOL_CFG_WIDTH2	BIT(2)

#define YTPHY_REG_SPACE_UTP             0
#define YTPHY_REG_SPACE_FIBER           2


static int yt8512_config(struct phy_device *phydev);
static int yt8512_clk_init(struct phy_device *phydev);
static int yt8512_led_init(struct phy_device *phydev);


static struct phy_driver YT8212B_driver =  {
	.name = "YT8512B Ethernet",
	.uid = PHY_ID_YT8512B,
	.mask = MOTORCOMM_PHY_ID_MASK,
	.features = PHY_BASIC_FEATURES,
	.config = yt8512_config,
	.startup = genphy_startup,
	.shutdown = genphy_shutdown,
};

static int ytphy_read_ext(struct phy_device *phydev, u32 regnum)
{
	int ret;
	int val;

	ret = phy_write(phydev, MDIO_DEVAD_NONE,REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, MDIO_DEVAD_NONE,REG_DEBUG_DATA);

	return val;
}

static int ytphy_write_ext(struct phy_device *phydev, u32 regnum, u16 val)
{
	int ret;

	ret = phy_write(phydev, MDIO_DEVAD_NONE,REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	ret = phy_write(phydev, MDIO_DEVAD_NONE,REG_DEBUG_DATA, val);

	return ret;
}


static int yt8512_config(struct phy_device *phydev)
{
	int ret;
	int val;



	//puts("yt8512 config init!\n");
	ret = yt8512_clk_init(phydev);
	if (ret < 0)
		return ret;

	ret = yt8512_led_init(phydev);

	/* disable auto sleep */
	val = ytphy_read_ext(phydev, YT8512_EXTREG_SLEEP_CONTROL1);
	if (val < 0)
		return val;

	val &= (~BIT(YT8512_EN_SLEEP_SW_BIT));

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_SLEEP_CONTROL1, val);
	if (ret < 0)
		return ret;


	ret = genphy_config_aneg(phydev);
	ret = genphy_restart_aneg(phydev);


	//puts("yt8512 config exit!\n");

	return ret;
}

static int yt8512_clk_init(struct phy_device *phydev)
{
	int ret;
	int val;

	val = ytphy_read_ext(phydev, YT8512_EXTREG_AFE_PLL);
	if (val < 0)
		return val;

	val |= YT8512_CONFIG_PLL_REFCLK_SEL_EN;

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_AFE_PLL, val);
	if (ret < 0)
		return ret;

	val = ytphy_read_ext(phydev, YT8512_EXTREG_EXTEND_COMBO);
	if (val < 0)
		return val;

	val |= YT8512_CONTROL1_RMII_EN;

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_EXTEND_COMBO, val);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, MDIO_DEVAD_NONE,MII_BMCR);
	if (val < 0)
		return val;

	val |= YT_SOFTWARE_RESET;
	ret = phy_write(phydev, MDIO_DEVAD_NONE,MII_BMCR, val);

	return ret;
}

static int yt8512_led_init(struct phy_device *phydev)
{
	int ret;
	int val;
	int mask;

	val = ytphy_read_ext(phydev, YT8512_EXTREG_LED0);
	if (val < 0)
		return val;

	val |= YT8512_LED0_ACT_BLK_IND;

	mask = YT8512_LED0_DIS_LED_AN_TRY | YT8512_LED0_BT_BLK_EN |
		YT8512_LED0_HT_BLK_EN | YT8512_LED0_COL_BLK_EN |
		YT8512_LED0_BT_ON_EN;
	val &= ~mask;

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_LED0, val);
	if (ret < 0)
		return ret;

	val = ytphy_read_ext(phydev, YT8512_EXTREG_LED1);
	if (val < 0)
		return val;

	val |= YT8512_LED1_BT_ON_EN;

	mask = YT8512_LED1_TXACT_BLK_EN | YT8512_LED1_RXACT_BLK_EN;
	val &= ~mask;

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_LED1, val);

	return ret;
}


int phy_motorcomm_init(void)
{
	phy_register(&YT8212B_driver);


	return 0;
}

