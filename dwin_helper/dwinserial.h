/*
 * dwinserial.h
 *
 *  Created on: 1 мар. 2023 г.
 *      Author: sergei
 */

#ifndef DWINSERIAL_H_
#define DWINSERIAL_H_

#include "serial.h"

/*dwin serial protocol*/
//#define BUFSIZE	100

enum STATE{
	IDLE,
	SET_BACKLIGHT,
	UPDATE_RTC,
};

enum DATA_TYPE{
	CLOCK,
	TOUCH
};

class dwin_serial: public serial_client {
	STATE state;
	uint backlight_brightnes;
	uint touch_x, touch_y;
	bool touch_touched;
	uint year, month, day, dow, hour, minute, sec;

	bool is_format_ok(uint8_t *buffer, uint length);
	uint8_t calc_crc(uint8_t *buf, uint len);
	uint set_rtc_request(uint8_t *buffer, uint bufsize);
	uint set_backlight_request(uint8_t *buffer, uint bufsize);
public:
	dwin_serial();
	virtual ~dwin_serial();
	void (*on_data_recived)(dwin_serial *handler, DATA_TYPE data_type);
	uint estimated_resp_size(uint8_t *buffer, uint length);
	// формирует запрос. возвращает длину запроса
	int request(int retry, uint8_t *buffer, uint bufsize);
	// разбор ответа
	void result (uint8_t *buffer, uint length);

	void update_rtc()	{state = UPDATE_RTC;}
	void set_backlight(uint brightnes=100)	{state = SET_BACKLIGHT; backlight_brightnes = brightnes;}


	uint get_x()	{return touch_x;}
	uint get_y()	{return touch_y;}
	bool is_touched(){return touch_touched;}

	uint getDay() const {return day;}
	uint getHour() const {return hour;}
	uint getMinute() const {return minute;}
	uint getMonth() const {	return month;}
	uint getSec() const {return sec;}
	uint getWeekDay() const {return dow;}
	uint getYear() const {return year;}

};

#endif /* DWINSERIAL_H_ */
