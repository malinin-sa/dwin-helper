/*
 * dwinserial.cpp
 *
 *  Created on: 1 мар. 2023 г.
 *      Author: sergei
 */

#include "dwinserial.h"
#include "time.h"
#include <cstring>

dwin_serial::dwin_serial() {
	state = IDLE;
	on_data_recived = NULL;
	touch_touched = false;
	touch_x = touch_y = 0;
	backlight_brightnes = 100;
}

dwin_serial::~dwin_serial() {
	// TODO Auto-generated destructor stub
}

int dwin_serial::request(int retry, uint8_t *buffer, uint bufsize) {
//	printf("\ndwin_serial: request, state %i", state);
	switch (state) {
	case IDLE:
		return 0;
	case SET_BACKLIGHT:
		printf("\ndwin_serial: Update RTC");
		state = IDLE;
		return set_backlight_request(buffer, bufsize);
	case UPDATE_RTC:
		printf("\ndwin_serial: Update RTC");
		state = IDLE;
		return set_rtc_request(buffer, bufsize);
	}
	return 0;
}

uint dwin_serial::set_backlight_request(uint8_t *buf, uint bufsize) {
	uint i=0;
	if (bufsize < 6)
		return 0;
	buf[i++] = 0x5a;
	buf[i++] = 0xa5;
	buf[i++] = 0x03;
	buf[i++] = 0x04;
	buf[i++] = backlight_brightnes > 100 ? 100 : backlight_brightnes;
	buf[i++] = calc_crc(buf+2, i-2);
	return i;
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

//создает запрос на запись текущего системного времени в RTC
uint dwin_serial::set_rtc_request(uint8_t *buf, uint bufsize){
	if (bufsize < 11)
		return 0;
/*
	//set time 16.04.2023 23:41:55
	uint8_t test[] = {0x5a, 0xa5, 0x08, 0x02, 0x17, 0x03, 0x10, 0x17, 0x29, 0x37, 0xab};
	memcpy(buf, test, sizeof(test));
	return sizeof(test);
*/
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	struct tm *my_tm = localtime(&ts.tv_sec);
	buf[0] = 0x5a;
	buf[1] = 0xa5;
	buf[2] = 0x08;
	buf[3] = 0x02;
	buf[4] = (uint8_t)my_tm->tm_year-100;
	buf[5] = (uint8_t)my_tm->tm_mon;
	buf[6] = (uint8_t)my_tm->tm_mday;
	buf[7] = (uint8_t)my_tm->tm_hour;
	buf[8] = (uint8_t)my_tm->tm_min;
	buf[9] = (uint8_t)my_tm->tm_sec;
	buf[10] = calc_crc(buf+2, 10-2);
	return 11;
}



