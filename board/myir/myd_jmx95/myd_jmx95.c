// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 NXP
 */

#include <env.h>
#include <efi_loader.h>
#include <init.h>
#include <fdt_support.h>
#include <asm/arch/clock.h>
#include <usb.h>
#include "../common/tcpc.h"
#include <dwc3-uboot.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <power/regulator.h>
#include <scmi_agent.h>
#include <scmi_protocols.h>
#include "../dts/upstream/src/arm64/myir/imx95-power.h"
#include <asm/arch/imx-regs.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <i2c_eeprom.h>
#include <dm/uclass.h>
#include <dm/uclass-internal.h>
#include <net.h>
#include <timestamp.h>
#include <version.h>

#define MAX_ETHERNET   0x2

struct myir_eeprom_struct{
	unsigned int crc32;
	unsigned char pn[48];
	unsigned char sn[48];
	unsigned char mac0[32];
	unsigned char mac1[32];
} myir_eeprom;

static const uint32_t crc32Table[256] = {
	0x00000000L, 0xF26B8303L, 0xE13B70F7L, 0x1350F3F4L,
	0xC79A971FL, 0x35F1141CL, 0x26A1E7E8L, 0xD4CA64EBL,
	0x8AD958CFL, 0x78B2DBCCL, 0x6BE22838L, 0x9989AB3BL,
	0x4D43CFD0L, 0xBF284CD3L, 0xAC78BF27L, 0x5E133C24L,
	0x105EC76FL, 0xE235446CL, 0xF165B798L, 0x030E349BL,
	0xD7C45070L, 0x25AFD373L, 0x36FF2087L, 0xC494A384L,
	0x9A879FA0L, 0x68EC1CA3L, 0x7BBCEF57L, 0x89D76C54L,
	0x5D1D08BFL, 0xAF768BBCL, 0xBC267848L, 0x4E4DFB4BL,
	0x20BD8EDEL, 0xD2D60DDDL, 0xC186FE29L, 0x33ED7D2AL,
	0xE72719C1L, 0x154C9AC2L, 0x061C6936L, 0xF477EA35L,
	0xAA64D611L, 0x580F5512L, 0x4B5FA6E6L, 0xB93425E5L,
	0x6DFE410EL, 0x9F95C20DL, 0x8CC531F9L, 0x7EAEB2FAL,
	0x30E349B1L, 0xC288CAB2L, 0xD1D83946L, 0x23B3BA45L,
	0xF779DEAEL, 0x05125DADL, 0x1642AE59L, 0xE4292D5AL,
	0xBA3A117EL, 0x4851927DL, 0x5B016189L, 0xA96AE28AL,
	0x7DA08661L, 0x8FCB0562L, 0x9C9BF696L, 0x6EF07595L,
	0x417B1DBCL, 0xB3109EBFL, 0xA0406D4BL, 0x522BEE48L,
	0x86E18AA3L, 0x748A09A0L, 0x67DAFA54L, 0x95B17957L,
	0xCBA24573L, 0x39C9C670L, 0x2A993584L, 0xD8F2B687L,
	0x0C38D26CL, 0xFE53516FL, 0xED03A29BL, 0x1F682198L,
	0x5125DAD3L, 0xA34E59D0L, 0xB01EAA24L, 0x42752927L,
	0x96BF4DCCL, 0x64D4CECFL, 0x77843D3BL, 0x85EFBE38L,
	0xDBFC821CL, 0x2997011FL, 0x3AC7F2EBL, 0xC8AC71E8L,
	0x1C661503L, 0xEE0D9600L, 0xFD5D65F4L, 0x0F36E6F7L,
	0x61C69362L, 0x93AD1061L, 0x80FDE395L, 0x72966096L,
	0xA65C047DL, 0x5437877EL, 0x4767748AL, 0xB50CF789L,
	0xEB1FCBADL, 0x197448AEL, 0x0A24BB5AL, 0xF84F3859L,
	0x2C855CB2L, 0xDEEEDFB1L, 0xCDBE2C45L, 0x3FD5AF46L,
	0x7198540DL, 0x83F3D70EL, 0x90A324FAL, 0x62C8A7F9L,
	0xB602C312L, 0x44694011L, 0x5739B3E5L, 0xA55230E6L,
	0xFB410CC2L, 0x092A8FC1L, 0x1A7A7C35L, 0xE811FF36L,
	0x3CDB9BDDL, 0xCEB018DEL, 0xDDE0EB2AL, 0x2F8B6829L,
	0x82F63B78L, 0x709DB87BL, 0x63CD4B8FL, 0x91A6C88CL,
	0x456CAC67L, 0xB7072F64L, 0xA457DC90L, 0x563C5F93L,
	0x082F63B7L, 0xFA44E0B4L, 0xE9141340L, 0x1B7F9043L,
	0xCFB5F4A8L, 0x3DDE77ABL, 0x2E8E845FL, 0xDCE5075CL,
	0x92A8FC17L, 0x60C37F14L, 0x73938CE0L, 0x81F80FE3L,
	0x55326B08L, 0xA759E80BL, 0xB4091BFFL, 0x466298FCL,
	0x1871A4D8L, 0xEA1A27DBL, 0xF94AD42FL, 0x0B21572CL,
	0xDFEB33C7L, 0x2D80B0C4L, 0x3ED04330L, 0xCCBBC033L,
	0xA24BB5A6L, 0x502036A5L, 0x4370C551L, 0xB11B4652L,
	0x65D122B9L, 0x97BAA1BAL, 0x84EA524EL, 0x7681D14DL,
	0x2892ED69L, 0xDAF96E6AL, 0xC9A99D9EL, 0x3BC21E9DL,
	0xEF087A76L, 0x1D63F975L, 0x0E330A81L, 0xFC588982L,
	0xB21572C9L, 0x407EF1CAL, 0x532E023EL, 0xA145813DL,
	0x758FE5D6L, 0x87E466D5L, 0x94B49521L, 0x66DF1622L,
	0x38CC2A06L, 0xCAA7A905L, 0xD9F75AF1L, 0x2B9CD9F2L,
	0xFF56BD19L, 0x0D3D3E1AL, 0x1E6DCDEEL, 0xEC064EEDL,
	0xC38D26C4L, 0x31E6A5C7L, 0x22B65633L, 0xD0DDD530L,
	0x0417B1DBL, 0xF67C32D8L, 0xE52CC12CL, 0x1747422FL,
	0x49547E0BL, 0xBB3FFD08L, 0xA86F0EFCL, 0x5A048DFFL,
	0x8ECEE914L, 0x7CA56A17L, 0x6FF599E3L, 0x9D9E1AE0L,
	0xD3D3E1ABL, 0x21B862A8L, 0x32E8915CL, 0xC083125FL,
	0x144976B4L, 0xE622F5B7L, 0xF5720643L, 0x07198540L,
	0x590AB964L, 0xAB613A67L, 0xB831C993L, 0x4A5A4A90L,
	0x9E902E7BL, 0x6CFBAD78L, 0x7FAB5E8CL, 0x8DC0DD8FL,
	0xE330A81AL, 0x115B2B19L, 0x020BD8EDL, 0xF0605BEEL,
	0x24AA3F05L, 0xD6C1BC06L, 0xC5914FF2L, 0x37FACCF1L,
	0x69E9F0D5L, 0x9B8273D6L, 0x88D28022L, 0x7AB90321L,
	0xAE7367CAL, 0x5C18E4C9L, 0x4F48173DL, 0xBD23943EL,
	0xF36E6F75L, 0x0105EC76L, 0x12551F82L, 0xE03E9C81L,
	0x34F4F86AL, 0xC69F7B69L, 0xD5CF889DL, 0x27A40B9EL,
	0x79B737BAL, 0x8BDCB4B9L, 0x988C474DL, 0x6AE7C44EL,
	0xBE2DA0A5L, 0x4C4623A6L, 0x5F16D052L, 0xAD7D5351L
};

static uint32_t singletable_crc32c(uint32_t crc, const void *buf, size_t size)
{
	const uint8_t *p = (uint8_t *)buf;

	while (size--)
		crc = crc32Table[(crc ^ *p++) & 0xff] ^ (crc >> 8);

	return crc;
}

// 检查字符是否为数字
int is_digit(char c) {
	return c >= '0' && c <= '9';
}

// 将大写字母转换为小写
char to_lower(char c) {
	if (c >= 'A' && c <= 'Z') {
		return c + 'a' - 'A';
	}
	return c;
}

// 字符转十六进制数字
int char_to_hex(int c) {
	if(is_digit(c))
		return c - '0';
	else
		return to_lower(c) - 'a' + 10;
}

// 字符串二进制转字节
unsigned char string_to_byte(const char* str) {
	return char_to_hex(str[0]) * 16 + char_to_hex(str[1]);
}

// MAC地址字符串转字节流
void mac_string_to_bytes( char *str, unsigned char *mac) {
	for(int i = 0; i < 6; ++i) {
		mac[i] = string_to_byte(str);
		str += 3; // 跳过冒号
	}
}

int myir_set_data_from_eeprom(void)
{
	int ret;
	struct udevice *dev;

	myir_eeprom.crc32=0x12345678;

	ret = uclass_first_device_err(UCLASS_I2C_EEPROM, &dev);
	if (ret)
		return ret;

	ret = i2c_eeprom_read(dev, 0, (uint8_t *)&myir_eeprom, sizeof(myir_eeprom));

	if(myir_eeprom.crc32 != singletable_crc32c(0x12345678,(uint8_t *)myir_eeprom.pn,sizeof(myir_eeprom)-4)){
		printf("check crc32 error!\n");
		return -1;
	}

	printf("\n");
	printf(">>>PN=%s\n",myir_eeprom.pn);
	printf(">>>SN=%s\n",myir_eeprom.sn);

	if(is_valid_ethaddr(myir_eeprom.mac0)){
		printf(">>>MAC0=%s\n",myir_eeprom.mac0);
		env_set("ethaddr", myir_eeprom.mac0);
	}
	if(is_valid_ethaddr(myir_eeprom.mac1)){
		printf(">>>MAC1=%s\n",myir_eeprom.mac1);
		env_set("eth1addr",myir_eeprom.mac1);
	}

	return 0;
}

extern int board_fix_fdt_fuse(void *fdt);

#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
#define IMX_BOOT_IMAGE_GUID \
	EFI_GUID(0x2c4db6b3, 0x0b15, 0x4a36, 0xbe, 0xae, \
		 0x1e, 0xa1, 0x35, 0x46, 0x4f, 0x5b)

struct efi_fw_image fw_images[] = {
	{
		.image_type_id = IMX_BOOT_IMAGE_GUID,
		.fw_name = u"IMX95-EVK-RAW",
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 0=flash-bin raw 0 0x2000 mmcpart 1",
	.num_images = ARRAY_SIZE(fw_images),
	.images = fw_images,
};
#endif /* EFI_HAVE_CAPSULE_SUPPORT */

int board_early_init_f(void)
{
	/* UART1: A55, UART2: M33 */
	init_uart_clk(0);

	return 0;
}

#ifdef CONFIG_USB_DWC3

#define PHY_CTRL0			0xF0040
#define PHY_CTRL0_REF_SSP_EN		BIT(2)
#define PHY_CTRL0_FSEL_MASK		GENMASK(10, 5)
#define PHY_CTRL0_FSEL_24M		0x2a
#define PHY_CTRL0_FSEL_100M		0x27
#define PHY_CTRL0_SSC_RANGE_MASK	GENMASK(23, 21)
#define PHY_CTRL0_SSC_RANGE_4003PPM	(0x2 << 21)

#define PHY_CTRL1			0xF0044
#define PHY_CTRL1_RESET			BIT(0)
#define PHY_CTRL1_COMMONONN		BIT(1)
#define PHY_CTRL1_ATERESET		BIT(3)
#define PHY_CTRL1_DCDENB		BIT(17)
#define PHY_CTRL1_CHRGSEL		BIT(18)
#define PHY_CTRL1_VDATSRCENB0		BIT(19)
#define PHY_CTRL1_VDATDETENB0		BIT(20)

#define PHY_CTRL2			0xF0048
#define PHY_CTRL2_TXENABLEN0		BIT(8)
#define PHY_CTRL2_OTG_DISABLE		BIT(9)

#define PHY_CTRL6			0xF0058
#define PHY_CTRL6_RXTERM_OVERRIDE_SEL	BIT(29)
#define PHY_CTRL6_ALT_CLK_EN		BIT(1)
#define PHY_CTRL6_ALT_CLK_SEL		BIT(0)

static struct dwc3_device dwc3_device_data = {
	.maximum_speed = USB_SPEED_HIGH,
	.base = USB1_BASE_ADDR,
	.dr_mode = USB_DR_MODE_PERIPHERAL,
	.index = 0,
	.power_down_scale = 2,
};

static void dwc3_nxp_usb_phy_init(struct dwc3_device *dwc3)
{
	u32 value;

	/* USB3.0 PHY signal fsel for 24M ref */
	value = readl(dwc3->base + PHY_CTRL0);
	value &= ~PHY_CTRL0_FSEL_MASK;
	value |= FIELD_PREP(PHY_CTRL0_FSEL_MASK, PHY_CTRL0_FSEL_24M);
	writel(value, dwc3->base + PHY_CTRL0);

	/* Disable alt_clk_en and use internal MPLL clocks */
	value = readl(dwc3->base + PHY_CTRL6);
	value &= ~(PHY_CTRL6_ALT_CLK_SEL | PHY_CTRL6_ALT_CLK_EN);
	writel(value, dwc3->base + PHY_CTRL6);

	value = readl(dwc3->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0);
	value |= PHY_CTRL1_RESET | PHY_CTRL1_ATERESET;
	writel(value, dwc3->base + PHY_CTRL1);

	value = readl(dwc3->base + PHY_CTRL0);
	value |= PHY_CTRL0_REF_SSP_EN;
	writel(value, dwc3->base + PHY_CTRL0);

	value = readl(dwc3->base + PHY_CTRL2);
	value |= PHY_CTRL2_TXENABLEN0 | PHY_CTRL2_OTG_DISABLE;
	writel(value, dwc3->base + PHY_CTRL2);

	udelay(10);

	value = readl(dwc3->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_RESET | PHY_CTRL1_ATERESET);
	writel(value, dwc3->base + PHY_CTRL1);
}
#endif

static int imx9_scmi_power_domain_enable(u32 domain, bool enable)
{
	struct udevice *dev;
	int ret;

	ret = uclass_get_device_by_name(UCLASS_CLK, "protocol@14", &dev);
	if (ret)
		return ret;

	return scmi_pwd_state_set(dev, 0, domain, enable ? 0 : BIT(30));
}

int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;

	if (index == 0 && init == USB_INIT_DEVICE) {
		ret = imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, true);
		if (ret) {
			printf("SCMI_POWWER_STATE_SET Failed for USB\n");
			return ret;
		}

#ifdef CONFIG_USB_DWC3
		dwc3_nxp_usb_phy_init(&dwc3_device_data);
#endif
#ifdef CONFIG_USB_DWC3
		return dwc3_uboot_init(&dwc3_device_data);
#endif
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;
	if (index == 0 && init == USB_INIT_DEVICE) {
#ifdef CONFIG_USB_DWC3
		dwc3_uboot_exit(index);
#endif
	}

	return ret;
}

static void netc_phy_rst(const char *gpio_name, const char *label)
{
	int ret;
	struct gpio_desc desc;

	/* ENET_RST_B */
	ret = dm_gpio_lookup_name(gpio_name, &desc);
	if (ret) {
		printf("%s lookup %s failed ret = %d\n", __func__, gpio_name, ret);
		return;
	}

	ret = dm_gpio_request(&desc, label);
	if (ret) {
		printf("%s request %s failed ret = %d\n", __func__, label, ret);
		return;
	}

	/* assert the ENET_RST_B */
	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE | GPIOD_ACTIVE_LOW);
	udelay(10000);
	dm_gpio_set_value(&desc, 1); /* deassert the ENET_RST_B */
	udelay(10000);
	dm_gpio_set_value(&desc, 0);
	udelay(10000);
	dm_gpio_set_value(&desc, 1);
	udelay(80000);

}

void netc_init(void)
{
	int ret;
	struct gpio_desc desc_phy1_reset; 
	struct gpio_desc desc_phy2_reset; 

	ret = imx9_scmi_power_domain_enable(IMX95_PD_NETC, false);
	udelay(10000);

	/* Power up the NETC MIX. */
	ret = imx9_scmi_power_domain_enable(IMX95_PD_NETC, true);
	if (ret) {
		printf("SCMI_POWWER_STATE_SET Failed for NETC MIX\n");
		return;
	}

#define PHY1_RESET "gpio@20_6"
#define PHY1_RESET_LABEL "ENET1_RST_B"

#define PHY2_RESET "GPIO2_30"
#define PHY2_RESET_LABEL "ENET2_RST_B"


	ret = dm_gpio_lookup_name(PHY1_RESET, &desc_phy1_reset);
	if (ret) {
		printf("%s lookup %s failed ret = %d\n", __func__, PHY1_RESET, ret);
		return;
	}
	ret = dm_gpio_request(&desc_phy1_reset, PHY1_RESET_LABEL);
	if (ret) {
		printf("%s request %s failed ret = %d\n", __func__, PHY1_RESET_LABEL, ret);
		return;
	}
	

	ret = dm_gpio_lookup_name(PHY2_RESET, &desc_phy2_reset);
	if (ret) {
		printf("%s lookup %s failed ret = %d\n", __func__, PHY2_RESET, ret);
		return;
	}
	ret = dm_gpio_request(&desc_phy2_reset, PHY2_RESET_LABEL);
	if (ret) {
		printf("%s request %s failed ret = %d\n", __func__, PHY2_RESET_LABEL, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc_phy1_reset, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	
	dm_gpio_set_dir_flags(&desc_phy2_reset, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE | GPIOD_ACTIVE_LOW);


	dm_gpio_set_value(&desc_phy1_reset, 1);
	dm_gpio_set_value(&desc_phy2_reset, 1);
	udelay(10000);
	dm_gpio_set_value(&desc_phy1_reset, 0);
	dm_gpio_set_value(&desc_phy2_reset, 0);
	udelay(10000);
	dm_gpio_set_value(&desc_phy1_reset, 1);
	dm_gpio_set_value(&desc_phy2_reset, 1);
	//udelay(80000);



	//netc_phy_rst("GPIO1_14", "ENET1_RST_B");
	//netc_phy_rst("GPIO2_30", "ENET2_RST_B");

	pci_init();
}

static void myir_board_init(void)
{
	struct gpio_desc desc;
	int ret;

	if (!IS_ENABLED(CONFIG_TARGET_MYD_JMX95_15X15))
		return;

	/* USB_HUB_RST */
	ret = dm_gpio_lookup_name("gpio@20_1", &desc);
	if (ret)
		return;

	ret = dm_gpio_request(&desc, "USB_HUB_RST");
	if (ret)
		return;

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 1);
	udelay(10000);
	dm_gpio_set_value(&desc, 0);
	udelay(10000);
	dm_gpio_set_value(&desc, 1);
}

int board_init(void)
{
	int ret;
	ret = imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, true);
	if (ret) {
		printf("SCMI_POWWER_STATE_SET Failed for USB\n");
		return ret;
	}

	imx9_scmi_power_domain_enable(IMX95_PD_DISPLAY, false);
	imx9_scmi_power_domain_enable(IMX95_PD_CAMERA, false);

	netc_init();

	myir_board_init();

	power_on_m7("mx95evkrpmsg");

	return 0;
}

/* i.MX95 NPU power domain ID (matches SM firmware DEV_SM_PD_NPU) */
#define IMX95_PD_NPU   20
/*
 * Check if NPU domain is powered via SCMI power domain protocol.
 * NPU SRAM (CONFIG_SAVED_QB_STATE_BASE) is only accessible when the
 * NPU domain is powered on. On warm reboot, SM may report POR as the
 * reset reason even though the NPU domain was powered down, causing a
 * Synchronous Abort when accessing NPU SRAM. This function provides a
 * reliable check of the actual NPU power state.
 */
static bool is_npu_powered(void)
{
	u32 domain_id = IMX95_PD_NPU;
	struct {
		s32 status;
		u32 pstate;
	} out = { 0 };
	struct scmi_msg msg = {
		.protocol_id = SCMI_PROTOCOL_ID_POWER_DOMAIN,
		.message_id = SCMI_PWD_STATE_GET,
		.in_msg = (u8 *)&domain_id,
		.in_msg_sz = sizeof(domain_id),
		.out_msg = (u8 *)&out,
		.out_msg_sz = sizeof(out),
	};
	struct udevice *dev;
	int ret;

	ret = uclass_get_device_by_name(UCLASS_CLK, "protocol@14", &dev);
	if (ret) {
		printf("is_npu_powered: SCMI dev not found, ret=%d\n", ret);
		return false;
	}

	ret = devm_scmi_process_msg(dev, &msg);
	if (ret) {
		printf("is_npu_powered: SCMI msg failed, ret=%d\n", ret);
		return false;
	}
	if (out.status) {
		printf("is_npu_powered: SCMI status=0x%x\n", out.status);
		return false;
	}

	printf("is_npu_powered: NPU pstate=0x%08x (0x0=ON, 0x40000000=OFF)\n", out.pstate);
	return (out.pstate == 0);
}

int board_late_init(void)
{
	if (IS_ENABLED(CONFIG_ENV_IS_IN_MMC))
		board_late_mmc_env_init();

	env_set("sec_boot", "no");
#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#endif
	myir_set_data_from_eeprom();

        /* Auto-save DDR QuickBoot training data on new training */
#ifdef CONFIG_CMD_QB
        {
                extern bool qb_check(void);
                extern int do_qb_save(struct cmd_tbl *, int, int, char * const []);

                /* Check actual NPU domain power state before accessing
                 * NPU SRAM. SCMI reset reason alone is unreliable because
                 * SM may report POR for Linux warm reboot even when the
                 * NPU domain is powered down. */
                if (!is_npu_powered()) {
                        printf("DDR QB: NPU domain not powered, skipping NPU SRAM access\n");
                } else {
                        volatile uint32_t *trained_flag =
                                (volatile uint32_t *)(CONFIG_SAVED_QB_STATE_BASE - 8);

                        if (qb_check() && (*trained_flag == 0xDEADBEEFU)) {
                                printf("DDR QB: new training data detected, auto-saving to flash...\n");
                                do_qb_save(NULL, 0, 0, NULL);
                                printf("DDR QB: auto-save done, next boot will use QuickBoot\n");
                        }
                        printf("DDR QB: trained_flag=0x%08x\n",*trained_flag);
                }
        }
#endif /* CONFIG_CMD_QB */

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
	char *p, *b, *s;
	char *token = NULL;
	int i, ret = 0;
	u64 base[CONFIG_NR_DRAM_BANKS] = {0};
	u64 size[CONFIG_NR_DRAM_BANKS] = {0};

	p = env_get("jh_root_mem");
	if (!p)
		return 0;

	i = 0;
	token = strtok(p, ",");
	while (token) {
		if (i >= CONFIG_NR_DRAM_BANKS) {
			printf("Error: The number of size@base exceeds CONFIG_NR_DRAM_BANKS.\n");
			return -EINVAL;
		}

		b = token;
		s = strsep(&b, "@");
		if (!s) {
			printf("The format of jh_root_mem is size@base[,size@base...].\n");
			return -EINVAL;
		}
		base[i] = simple_strtoull(b, NULL, 16);
		size[i] = simple_strtoull(s, NULL, 16);
		token = strtok(NULL, ",");
		i++;
	}

	ret = fdt_fixup_memory_banks(blob, base, size, CONFIG_NR_DRAM_BANKS);
	if (ret)
		return ret;

	return 0;
}
#endif

void board_quiesce_devices(void)
{
	int ret;
	struct uclass *uc_dev;

	ret = imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, false);
	if (ret) {
		printf("%s: Failed for HSIO MIX: %d\n", __func__, ret);
		return;
	}

	ret = imx9_scmi_power_domain_enable(IMX95_PD_NETC, false);
	if (ret) {
		printf("%s: Failed for NETC MIX: %d\n", __func__, ret);
		return;
	}

	ret = uclass_get(UCLASS_SPI_FLASH, &uc_dev);
	if (uc_dev)
		ret = uclass_destroy(uc_dev);
	if (ret)
		printf("couldn't remove SPI FLASH devices\n");
}

#if IS_ENABLED(CONFIG_OF_BOARD_FIXUP)

int board_fix_fdt(void *fdt)
{
	/* Remove nodes based on fuses. */
	board_fix_fdt_fuse(fdt);
	return 0;
}
#endif
#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0;
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/
