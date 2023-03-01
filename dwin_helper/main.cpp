/*
 * main.c
 *
 *  Created on: 28 февр. 2023 г.
 *      Author: sergei
 */
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "touch.h"
#include "serial.h"
#include "dwinserial.h"
#include "gpiopoll.h"
#include "time.h"

#define MODEM "/dev/ttyS5"
#define BAUDRATE B115200

touch ts;
bool time_is_set=false;

void set_time(uint Year, uint Month, uint Day, uint Hour, uint Minute, uint Second){
	struct tm time = { 0 };

	time.tm_year = Year - 1900;
	time.tm_mon  = Month - 1;
	time.tm_mday = Day;
	time.tm_hour = Hour;
	time.tm_min  = Minute;
	time.tm_sec  = Second;

	if (time.tm_year < 0) {
		time.tm_year = 0;
	}

	time_t t = mktime(&time);
	timespec  s_time = {t,0};

	printf("\n Update system time");
	clock_settime(CLOCK_REALTIME, &s_time);
	time_is_set = true;
}

void process_data(dwin_serial *instance, DATA_TYPE type){
	switch(type){
	case CLOCK:
		printf("\n %i.%i.%i %i:%i:%i", instance->getYear(), instance->getMonth(), instance->getDay(),
				instance->getHour(), instance->getMinute(), instance->getSec());
		if (!time_is_set)
			set_time(instance->getYear(), instance->getMonth(), instance->getDay(),
				instance->getHour(), instance->getMinute(), instance->getSec());
		break;
	case TOUCH:
		ts.event(instance->get_x(), instance->get_y(), instance->is_touched());
		break;
	}
}



int main(int argc,char** argv){
	serial *port = new serial(MODEM, 115200, 'n', true, false, 100, 10);
//	port->debug = true;
	dwin_serial *ds = new dwin_serial();
	port->add_client(ds);
	ds->on_data_recived = process_data;

	while (1) {
		gpio_polling_task();
	}
	return EXIT_SUCCESS;
}
