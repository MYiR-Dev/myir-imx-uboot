// SPDX-License-Identifier: MIT
#include <common.h>
#include <env.h>
#include <i2c.h>
#include <i2c_eeprom.h>
#include <net.h>
#include <eeprom.h>
#include <eeprom_layout.h>
#include "i2c_eeprom_myir.h"
#include <linux/delay.h>
static struct id_eeprom eeprom;

int show_eeprom(void)
{
	char safe_string[64] = {0};
	char sn[64];
	int i, len;
	u8 *p;
	u8 *m;
	char buf[ARP_HLEN_ASCII + 1];
	char buf1[ARP_HLEN_ASCII + 1];

	puts("Module INFO:\n");
	/* pn */
	len = strlen(eeprom.pn);
	if (len >= sizeof(eeprom.pn))//eliminate eeprom initial values
		return -1;
	memcpy(safe_string, eeprom.pn, len);
	safe_string[len] = '\0';
	if (len > 0) {
		printf(">>>PN=%s\n", safe_string);
		env_set("PN", safe_string);
	} else {
		puts("unknown hardware variant\n");
	}
	/* Serial number */
	len = strlen(eeprom.sn);
	if (len >= sizeof(eeprom.sn))
		return -1;
	memcpy(sn, eeprom.sn, len);
	sn[len] = '\0';
	if (len > 0) {
		printf(">>>SN=%s\n", sn);
		env_set("SN", sn);
	} else {
		puts("nunknown serial number\n");
	}
	/* MAC address  */
	p = eeprom.mac0;
	if (!is_valid_ethaddr(p)) {
		printf("Not valid MAC address in eeprom!\n");
		return 0;
	}
	printf(">>>MAC0=%pM\n", p);
	sprintf(buf, "%pM", p);
	env_set("ethaddr", buf);
	m = eeprom.mac1;
	if (!is_valid_ethaddr(m)) {
		printf("Not valid MAC1 address in eeprom!\n");
		return 0;
	}
	printf(">>>MAC1=%pM\n", m);
	sprintf(buf1, "%pM", m);
	env_set("eth1addr", buf1);
	return 0;
}

int read_eeprom(void)
{
	struct udevice *dev;
	int ret = 0;
	//at24LC32
	mdelay(200);
	ret = i2c_get_chip_for_busnum(0, 0x50, 2, &dev);
	if (ret) {
		printf("Cannot find EEPROM !\n");
		return ret;
	}
	i2c_set_chip_offset_len(dev, 2);
	ret = dm_i2c_read(dev, 0x00, (uchar *)&eeprom, sizeof(eeprom));
	if (ret)
		printf("Read eeprom fail !\n");
	return ret;
}
