// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2024 MYIR
 *
 * U-Boot driver for the MYIR MIPI101C MIPI DSI panel.
 */

#include <backlight.h>
#include <dm.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <asm/gpio.h>
#include <linux/delay.h>

struct mipi101c_cmd {
	u8 cmd;
	u8 param;
};

static const struct mipi101c_cmd mipi101c_init[] = {
	{ 0xb0, 0x5a }, { 0xb1, 0x00 }, { 0x89, 0x01 },
	{ 0x2c, 0x28 }, { 0x00, 0xf1 }, { 0x11, 0x00 },
	{ 0x29, 0x00 },
};

struct mipi101c_priv {
	struct udevice *backlight;
	struct gpio_desc reset;
	struct gpio_desc mux[2];
	unsigned int lanes;
	enum mipi_dsi_pixel_format format;
	unsigned long mode_flags;
};

static int mipi101c_push_init(struct mipi_dsi_device *dsi)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(mipi101c_init); i++) {
		ret = mipi_dsi_generic_write(dsi, &mipi101c_init[i],
					     sizeof(mipi101c_init[i]));
		if (ret < 0)
			return ret;
		udelay(100);
	}

	return 0;
}

static int mipi101c_enable(struct udevice *dev)
{
	struct mipi101c_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *dsi = plat->device;
	int ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi101c_push_init(dsi);
	if (ret < 0)
		return ret;

	mdelay(15);

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_set_tear_scanline(dsi, 0);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_set_pixel_format(dsi, 0x77);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		return ret;

	mdelay(5);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0)
		return ret;

	if (priv->backlight)
		return backlight_enable(priv->backlight);

	return 0;
}

static int mipi101c_get_display_timing(struct udevice *dev,
				       struct display_timing *timing)
{
	struct mipi101c_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	int ret;

	ret = ofnode_decode_panel_timing(dev_ofnode(dev), timing);
	if (ret)
		return ret;

	if (plat->device) {
		plat->device->lanes = priv->lanes;
		plat->device->format = priv->format;
		plat->device->mode_flags = priv->mode_flags;
	}

	return 0;
}

static int mipi101c_probe(struct udevice *dev)
{
	struct mipi101c_priv *priv = dev_get_priv(dev);
	u32 video_mode;
	int count, i, ret;

	priv->format = MIPI_DSI_FMT_RGB888;
	priv->mode_flags = MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO |
			   MIPI_DSI_MODE_EOT_PACKET;

	ret = dev_read_u32(dev, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			priv->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
			break;
		case 1:
			break;
		case 2:
			priv->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
			break;
		default:
			return -EINVAL;
		}
	}

	ret = dev_read_u32(dev, "dsi-lanes", &priv->lanes);
	if (ret)
		return ret;

	count = gpio_request_list_by_name(dev, "mux-gpios", priv->mux,
					  ARRAY_SIZE(priv->mux), GPIOD_IS_OUT);
	if (count < 0)
		return count;

	for (i = 0; i < count; i++) {
		ret = dm_gpio_set_value(&priv->mux[i], true);
		if (ret)
			return ret;
	}

	ret = gpio_request_by_name(dev, "reset-gpios", 0, &priv->reset,
				   GPIOD_IS_OUT);
	if (ret)
		return ret;

	ret = dm_gpio_set_value(&priv->reset, true);
	if (ret)
		return ret;
	mdelay(20);

	ret = dm_gpio_set_value(&priv->reset, false);
	if (ret)
		return ret;
	mdelay(120);

	ret = uclass_get_device_by_phandle(UCLASS_PANEL_BACKLIGHT, dev,
					   "backlight", &priv->backlight);
	if (ret == -ENOENT)
		priv->backlight = NULL;
	else if (ret)
		return ret;

	return 0;
}

static int mipi101c_remove(struct udevice *dev)
{
	struct mipi101c_priv *priv = dev_get_priv(dev);

	return dm_gpio_set_value(&priv->reset, true);
}

static const struct panel_ops mipi101c_ops = {
	.enable_backlight = mipi101c_enable,
	.get_display_timing = mipi101c_get_display_timing,
};

static const struct udevice_id mipi101c_ids[] = {
	{ .compatible = "myir,mipi101c-imx8mp" },
	{ }
};

U_BOOT_DRIVER(mipi101c_panel) = {
	.name = "mipi101c_panel",
	.id = UCLASS_PANEL,
	.of_match = mipi101c_ids,
	.ops = &mipi101c_ops,
	.probe = mipi101c_probe,
	.remove = mipi101c_remove,
	.plat_auto = sizeof(struct mipi_dsi_panel_plat),
	.priv_auto = sizeof(struct mipi101c_priv),
};
