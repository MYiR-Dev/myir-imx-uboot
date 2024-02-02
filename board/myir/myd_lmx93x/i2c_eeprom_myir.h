/* SPDX-License-Identifier: MIT */
#ifndef __I2C_EEPROM_MYIR_H
#define __I2C_EEPROM_MYIR_H

struct id_eeprom {
	u8 pn[32];/*0x00 ... 0x1f*/
	u8 sn[32];/*0x20 ... 0x3f*/
	u8 mac0[8];/*0x40 ... 0x47*/
	u8 mac1[8];/*0x48 ... 0x4f*/
} __packed;

int show_eeprom(void);
int read_eeprom(void);
int read_board_id(void);//no user
#endif //__I2C_EEPROM_MYIR_H
