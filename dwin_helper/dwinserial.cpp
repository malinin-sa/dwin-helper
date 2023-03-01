/*
 * dwinserial.cpp
 *
 *  Created on: 1 мар. 2023 г.
 *      Author: sergei
 */

#include "dwinserial.h"

dwin_serial::dwin_serial() {
	// TODO Auto-generated constructor stub

}

dwin_serial::~dwin_serial() {
	// TODO Auto-generated destructor stub
}

int dwin_serial::request(int retry, uint8_t *buffer, uint bufsize) {
	return 0;
}

void dwin_serial::result(uint8_t *buffer, uint length) {
	printf("\ndwin_serial: recived %i bytes", length);
	if (!is_format_ok(buffer, length))
		return;

	switch(buffer[3]){
	case 0x03: //beep
		return;
	case 0x04:	//backlight
		return;
	case 0x11: //touch 5A A5 07 11 03 00 2d 00 56 9e
		touch_x = (buffer[5] << 8) | buffer[6];
		touch_y = (buffer[7] << 8) | buffer[8];
		touch_touched = (buffer[4] == 1 || buffer[4] == 3);
		if (on_data_recived)
			on_data_recived(this, TOUCH);
		break;
	case 0x12:	//clock 5A A5 0B 12 17 01 0E 06 00 10 20 00 01 7A
		year = buffer[4] >= 70 ? 1900 + buffer[4] : 2000 + buffer[4];
		month = buffer[5]+1;
		day = buffer[6];
		dow = buffer[7]+1;
		hour = buffer[8];
		minute = buffer[9];
		sec = buffer[10];
		if (on_data_recived)
			on_data_recived(this, CLOCK);
		break;
	}
}


uint8_t dwin_serial::calc_crc(uint8_t *buf, uint len) {
	uint8_t crc=0;
	for (uint i=0; i<len; i++){
//		printf(" %02X", buf[i]);
		crc +=buf[i];
	}
//	printf(" -> %02X", crc);
	return crc;
}

bool dwin_serial::is_format_ok(uint8_t *buffer, uint length) {
	if (length < 6)
		return false;
	if (buffer[0] != 0x5A && buffer[1] != 0xA5)
		return false;
	if (length < 3 + buffer[2])
		return false;
	if (calc_crc(buffer+2, buffer[2]) != buffer[buffer[2]+2])
		return false;
	return true;
}

uint dwin_serial::estimated_resp_size(uint8_t *buffer, uint length) {
	if (length < 3)
		return 0;
	if (buffer[0] != 0x5A && buffer[1] != 0xA5)
		return 0;
	return 3 + buffer[2];
}
