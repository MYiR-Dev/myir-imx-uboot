// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 NXP
 */

#include <asm/arch/clock.h>
#include <dm/uclass.h>
#include <scmi_agent.h>
#include <scmi_nxp_protocols.h>

#include "common.h"

u32 get_arm_core_clk(void)
{
	u32 val;

	val = imx_clk_scmi_get_rate(SCMI_CLK(SEL_A55C0));
	if (val)
		return val;
	return imx_clk_scmi_get_rate(SCMI_CLK(A55));
}

void enable_usboh3_clk(unsigned char enable)
{

}

int clock_init_early(void)
{
	return 0;
}

int clock_init_late(void)
{
	/* System Manager already sets the ARM CLK to max allowed. */

	return 0;
}

u32 get_lpuart_clk(void)
{
	return imx_clk_scmi_get_rate(SCMI_CLK(LPUART1));
}

void init_uart_clk(u32 index)
{
	u32 clock_id;

	switch (index) {
	case 0:
		clock_id = SCMI_CLK(LPUART1);
		break;
	case 1:
		clock_id = SCMI_CLK(LPUART2);
		break;
	case 2:
		clock_id = SCMI_CLK(LPUART3);
		break;
	default:
		return;
	}

	/* 24MHz */
	imx_clk_scmi_enable(clock_id, false);
	imx_clk_scmi_set_parent(clock_id, SCMI_CLK(24M));
	imx_clk_scmi_set_rate(clock_id, 24000000);
	imx_clk_scmi_enable(clock_id, true);
}

int set_clk_netc(enum enet_freq type)
{
	ulong rate;

	switch (type) {
	case ENET_125MHZ:
		rate = MHZ(250); /* 250Mhz */
		break;
	case ENET_50MHZ:
		rate = MHZ(100); /* 100Mhz */
		break;
	case ENET_25MHZ:
		rate = MHZ(50); /* 50Mhz */
		break;
	default:
		return -EINVAL;
	}

#if IS_ENABLED(CONFIG_IMX94)
	/* disable the clock first */
	imx_clk_scmi_enable(SCMI_CLK(MAC4), false);
	imx_clk_scmi_set_parent(SCMI_CLK(MAC4), SCMI_CLK(SYSPLL1_PFD0));
	imx_clk_scmi_set_rate(SCMI_CLK(MAC4), rate);
	imx_clk_scmi_enable(SCMI_CLK(MAC4), true);

	imx_clk_scmi_enable(SCMI_CLK(MAC5), false);
	imx_clk_scmi_set_parent(SCMI_CLK(MAC5), SCMI_CLK(SYSPLL1_PFD0));
	imx_clk_scmi_set_rate(SCMI_CLK(MAC5), rate);
	imx_clk_scmi_enable(SCMI_CLK(MAC5), true);
#else
	/* disable the clock first */
	imx_clk_scmi_enable(SCMI_CLK(ENETREF), false);
	imx_clk_scmi_set_parent(SCMI_CLK(ENETREF), SCMI_CLK(SYSPLL1_PFD0));
	imx_clk_scmi_set_rate(SCMI_CLK(ENETREF), rate);
	imx_clk_scmi_enable(SCMI_CLK(ENETREF), true);
#endif

	return 0;
}

unsigned int mxc_get_clock(enum mxc_clock clk)
{
	switch (clk) {
	case MXC_ARM_CLK:
		return get_arm_core_clk();
	case MXC_IPG_CLK:
		return imx_clk_scmi_get_rate(SCMI_CLK(BUSWAKEUP));
	case MXC_CSPI_CLK:
		return imx_clk_scmi_get_rate(SCMI_CLK(LPSPI1));
	case MXC_ESDHC_CLK:
		return imx_clk_scmi_get_rate(SCMI_CLK(USDHC1));
	case MXC_ESDHC2_CLK:
		return imx_clk_scmi_get_rate(SCMI_CLK(USDHC2));
	case MXC_ESDHC3_CLK:
		return imx_clk_scmi_get_rate(SCMI_CLK(USDHC3));
	case MXC_UART_CLK:
		return imx_clk_scmi_get_rate(SCMI_CLK(LPUART1));
	case MXC_FLEXSPI_CLK:
		return imx_clk_scmi_get_rate(SCMI_CLK(FLEXSPI1));
	default:
		return -1;
	};

	return -1;
};

static uint32_t clock_ids[] =
{
	SCMI_CLK(SAI1),
	SCMI_CLK(SAI2),
	SCMI_CLK(SAI3),
	SCMI_CLK(SAI4),
#ifdef CONFIG_IMX95
	SCMI_CLK(SAI5),
	SCMI_CLK(SPDIF),
#endif
	SCMI_CLK(PDM),
#ifdef CONFIG_IMX95
	SCMI_CLK(MQS1),
	SCMI_CLK(MQS2),
#endif
};

int board_prep_linux(struct bootm_headers *images)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(clock_ids); i++)
		imx_clk_scmi_enable(clock_ids[i], false);

	return 0;
}
