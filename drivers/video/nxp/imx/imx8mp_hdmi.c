// SPDX-License-Identifier: GPL-2.0+
/*
 * i.MX8M Plus native HDMI boot display
 *
 * Copyright 2026 MYIR Electronics
 *
 * This is intentionally limited to the board's 1920x1080p60 boot mode.
 * Linux takes over with EDID-based mode selection after boot.
 */

#include <dm.h>
#include <dm/device_compat.h>
#include <dm/device-internal.h>
#include <dw_hdmi.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <media_bus_format.h>
#include <video_bridge.h>
#include <video_link.h>
#include <asm/io.h>

#define HTX_PVI_CTRL			0x00
#define PVI_CTRL_OP_VSYNC_POL		BIT(18)
#define PVI_CTRL_OP_HSYNC_POL		BIT(17)
#define PVI_CTRL_OP_DE_POL		BIT(16)
#define PVI_CTRL_INP_VSYNC_POL		BIT(14)
#define PVI_CTRL_INP_HSYNC_POL		BIT(13)
#define PVI_CTRL_INP_DE_POL		BIT(12)
#define PVI_CTRL_MODE_LCDIF		(2 << 1)
#define PVI_CTRL_EN			BIT(0)

#define PHY_REG(n)			((n) * 4)
#define PHY_REG_33			PHY_REG(33)
#define PHY_REG_34			PHY_REG(34)
#define PHY_REG33_FIX_DA		BIT(1)
#define PHY_REG33_MODE_SET_DONE		BIT(7)
#define PHY_REG34_PLL_LOCK		BIT(6)

#define IMX8MP_HDMI_PIXEL_CLOCK		148500000

struct imx8mp_hdmi_priv {
	struct dw_hdmi hdmi;
	void __iomem *phy;
};

struct imx8mp_hdmi_pvi_priv {
	void __iomem *regs;
};

struct imx8mp_phy_reg {
	u8 reg;
	u8 val;
};

static const struct imx8mp_phy_reg imx8mp_phy_common[] = {
	{ 0, 0x00 }, { 1, 0xd1 }, { 8, 0x4f }, { 9, 0x30 },
	{ 10, 0x33 }, { 11, 0x65 }, { 15, 0x80 }, { 16, 0x6c },
	{ 17, 0xf2 }, { 18, 0x67 }, { 19, 0x00 }, { 20, 0x10 },
	{ 22, 0x30 }, { 23, 0x32 }, { 24, 0x60 }, { 25, 0x8f },
	{ 26, 0x00 }, { 27, 0x00 }, { 28, 0x08 }, { 29, 0x00 },
	{ 30, 0x00 }, { 31, 0x00 }, { 32, 0x00 }, { 33, 0x80 },
	{ 34, 0x00 }, { 35, 0x00 }, { 36, 0x00 }, { 37, 0x00 },
	{ 38, 0x00 }, { 39, 0x00 }, { 40, 0x00 }, { 41, 0xe0 },
	{ 42, 0x83 }, { 43, 0x0f }, { 44, 0x3e }, { 45, 0xf8 },
	{ 46, 0x00 }, { 47, 0x00 },
};

static const u8 imx8mp_phy_148500_pll[] = {
	0x7b, 0x35, 0x84, 0x03, 0x90, 0x45,
};

static const struct display_timing imx8mp_hdmi_1080p60 = {
	.pixelclock = { .typ = IMX8MP_HDMI_PIXEL_CLOCK },
	.hactive = { .typ = 1920 },
	.hfront_porch = { .typ = 88 },
	.hback_porch = { .typ = 148 },
	.hsync_len = { .typ = 44 },
	.vactive = { .typ = 1080 },
	.vfront_porch = { .typ = 4 },
	.vback_porch = { .typ = 36 },
	.vsync_len = { .typ = 5 },
	.flags = DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH,
	.hdmi_monitor = true,
};

static int imx8mp_hdmi_phy_set(struct dw_hdmi *hdmi, uint pixelclock)
{
	struct imx8mp_hdmi_priv *priv =
		container_of(hdmi, struct imx8mp_hdmi_priv, hdmi);
	u8 val;
	int i, ret;

	if (pixelclock != IMX8MP_HDMI_PIXEL_CLOCK)
		return -EINVAL;

	/* Release the external Gen1 PHY only after both PHY domains are on. */
	writeb(HDMI_MC_PHYRSTZ_ASSERT, hdmi->ioaddr + HDMI_MC_PHYRSTZ);
	writeb(HDMI_MC_PHYRSTZ_DEASSERT, hdmi->ioaddr + HDMI_MC_PHYRSTZ);

	writeb(PHY_REG33_FIX_DA, priv->phy + PHY_REG_33);
	for (i = 0; i < ARRAY_SIZE(imx8mp_phy_common); i++)
		writeb(imx8mp_phy_common[i].val,
		       priv->phy + PHY_REG(imx8mp_phy_common[i].reg));

	for (i = 0; i < ARRAY_SIZE(imx8mp_phy_148500_pll); i++)
		writeb(imx8mp_phy_148500_pll[i], priv->phy + PHY_REG(2 + i));

	/* Linux FLD settings for 148.5 MHz: divide by 4, target code 166. */
	writeb(0x20, priv->phy + PHY_REG(12));
	writeb(0xa6, priv->phy + PHY_REG(13));
	writeb(0x24, priv->phy + PHY_REG(14));
	writeb(0x83, priv->phy + PHY_REG(21));
	writeb(PHY_REG33_FIX_DA | PHY_REG33_MODE_SET_DONE,
	       priv->phy + PHY_REG_33);

	ret = readb_poll_timeout(priv->phy + PHY_REG_34, val,
				 val & PHY_REG34_PLL_LOCK, 20000);
	if (ret) {
		printf("i.MX8MP HDMI: PHY PLL did not lock\n");
		return ret;
	}

	mdelay(200);
	return 0;
}

static const struct dw_hdmi_phy_ops imx8mp_hdmi_phy_ops = {
	.phy_set = imx8mp_hdmi_phy_set,
};

static int imx8mp_hdmi_attach(struct udevice *dev)
{
	struct imx8mp_hdmi_priv *priv = dev_get_priv(dev);
	int ret;

	ret = dw_hdmi_enable(&priv->hdmi, &imx8mp_hdmi_1080p60);
	if (ret)
		dev_err(dev, "transmitter enable failed (%d)\n", ret);

	return ret;
}

static int imx8mp_hdmi_set_backlight(struct udevice *dev, int percent)
{
	return 0;
}

static int imx8mp_hdmi_probe(struct udevice *dev)
{
	struct imx8mp_hdmi_priv *priv = dev_get_priv(dev);
	struct dw_hdmi *hdmi = &priv->hdmi;
	ofnode phy_node;

	hdmi->ioaddr = dev_read_addr(dev);
	if (hdmi->ioaddr == FDT_ADDR_T_NONE)
		return -EINVAL;

	phy_node = ofnode_by_compatible(ofnode_null(), "fsl,imx8mp-hdmi-phy");
	if (!ofnode_valid(phy_node) || !ofnode_is_enabled(phy_node))
		return -ENODEV;

	priv->phy = (void __iomem *)ofnode_get_addr(phy_node);
	if ((fdt_addr_t)priv->phy == FDT_ADDR_T_NONE)
		return -EINVAL;

	hdmi->reg_io_width = 1;
	hdmi->i2c_clk_high = 0x67;
	hdmi->i2c_clk_low = 0x78;
	hdmi->ops = &imx8mp_hdmi_phy_ops;
	hdmi->hdmi_data.enc_in_bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	hdmi->hdmi_data.enc_out_bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	dw_hdmi_init(hdmi);
	return 0;
}

static int imx8mp_hdmi_remove(struct udevice *dev)
{
	struct imx8mp_hdmi_priv *priv = dev_get_priv(dev);
	struct dw_hdmi *hdmi = &priv->hdmi;

	/*
	 * Leave the transmitter and external PHY quiescent for the OS.  The
	 * device core turns off the HDMI TX and PHY power domains after this
	 * callback, so all register accesses must be completed here.
	 */
	writeb(HDMI_IH_MUTE_MUTE_WAKEUP_INTERRUPT |
	       HDMI_IH_MUTE_MUTE_ALL_INTERRUPT,
	       hdmi->ioaddr + HDMI_IH_MUTE);
	writeb(PHY_REG33_FIX_DA, priv->phy + PHY_REG_33);
	writeb(HDMI_MC_PHYRSTZ_ASSERT, hdmi->ioaddr + HDMI_MC_PHYRSTZ);
	writeb(HDMI_MC_CLKDIS_HDCPCLK_DISABLE |
	       HDMI_MC_CLKDIS_CECCLK_DISABLE |
	       HDMI_MC_CLKDIS_CSCCLK_DISABLE |
	       HDMI_MC_CLKDIS_AUDCLK_DISABLE |
	       HDMI_MC_CLKDIS_PREPCLK_DISABLE |
	       HDMI_MC_CLKDIS_TMDSCLK_DISABLE |
	       HDMI_MC_CLKDIS_PIXELCLK_DISABLE,
	       hdmi->ioaddr + HDMI_MC_CLKDIS);

	return 0;
}

static const struct video_bridge_ops imx8mp_hdmi_ops = {
	.attach = imx8mp_hdmi_attach,
	.set_backlight = imx8mp_hdmi_set_backlight,
};

static const struct udevice_id imx8mp_hdmi_ids[] = {
	{ .compatible = "fsl,imx8mp-hdmi-tx" },
	{ }
};

U_BOOT_DRIVER(imx8mp_hdmi) = {
	.name = "imx8mp_hdmi",
	.id = UCLASS_VIDEO_BRIDGE,
	.of_match = imx8mp_hdmi_ids,
	.probe = imx8mp_hdmi_probe,
	.remove = imx8mp_hdmi_remove,
	.ops = &imx8mp_hdmi_ops,
	.flags = DM_FLAG_OS_PREPARE,
	.priv_auto = sizeof(struct imx8mp_hdmi_priv),
};

static int imx8mp_hdmi_pvi_attach(struct udevice *dev)
{
	struct imx8mp_hdmi_pvi_priv *priv = dev_get_priv(dev);
	struct udevice *next;
	u32 val;
	int ret;

	next = video_link_get_next_device(dev);
	if (!next || device_get_uclass_id(next) != UCLASS_VIDEO_BRIDGE)
		return -ENODEV;

	ret = video_bridge_attach(next);
	if (ret)
		return ret;

	val = PVI_CTRL_MODE_LCDIF | PVI_CTRL_EN |
	      PVI_CTRL_OP_VSYNC_POL | PVI_CTRL_OP_HSYNC_POL |
	      PVI_CTRL_OP_DE_POL | PVI_CTRL_INP_VSYNC_POL |
	      PVI_CTRL_INP_HSYNC_POL | PVI_CTRL_INP_DE_POL;
	writel(val, priv->regs + HTX_PVI_CTRL);

	return 0;
}

static int imx8mp_hdmi_pvi_set_backlight(struct udevice *dev, int percent)
{
	struct udevice *next = video_link_get_next_device(dev);

	if (!next || device_get_uclass_id(next) != UCLASS_VIDEO_BRIDGE)
		return -ENODEV;

	return video_bridge_set_backlight(next, percent);
}

static int imx8mp_hdmi_pvi_probe(struct udevice *dev)
{
	struct imx8mp_hdmi_pvi_priv *priv = dev_get_priv(dev);

	priv->regs = dev_read_addr_ptr(dev);
	if (!priv->regs)
		return -EINVAL;

	return 0;
}

static int imx8mp_hdmi_pvi_remove(struct udevice *dev)
{
	struct imx8mp_hdmi_pvi_priv *priv = dev_get_priv(dev);
	struct udevice *next;
	int ret = 0;

	/* Stop forwarding pixels before the HDMI power domains are removed. */
	writel(0, priv->regs + HTX_PVI_CTRL);

	/* The graph-connected transmitter is not a child in the DM tree. */
	next = video_link_get_next_device(dev);
	if (next)
		ret = device_remove(next, DM_REMOVE_NORMAL);

	return ret;
}

static const struct video_bridge_ops imx8mp_hdmi_pvi_ops = {
	.attach = imx8mp_hdmi_pvi_attach,
	.set_backlight = imx8mp_hdmi_pvi_set_backlight,
};

static const struct udevice_id imx8mp_hdmi_pvi_ids[] = {
	{ .compatible = "fsl,imx8mp-hdmi-pvi" },
	{ }
};

U_BOOT_DRIVER(imx8mp_hdmi_pvi) = {
	.name = "imx8mp_hdmi_pvi",
	.id = UCLASS_VIDEO_BRIDGE,
	.of_match = imx8mp_hdmi_pvi_ids,
	.probe = imx8mp_hdmi_pvi_probe,
	.remove = imx8mp_hdmi_pvi_remove,
	.ops = &imx8mp_hdmi_pvi_ops,
	.flags = DM_FLAG_OS_PREPARE,
	.priv_auto = sizeof(struct imx8mp_hdmi_pvi_priv),
};
