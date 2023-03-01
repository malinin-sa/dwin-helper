/*
 * gpiopoll.h
 *
 *  Created on: 12.11.2013
 *      Author: user
 */

#ifndef GPIOPOLL_H_
#define GPIOPOLL_H_

#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include "timer.h"

typedef void* timer_p;

struct gpio_info_t
{
	void 	(* gpio_change_handler_func)(uint8_t exportn, uint8_t value, void * param);
	void 	*handler_param;
	int		fd;
	uint8_t export_pin;
	uint8_t value;
};

/*
 * класс служит для взаимодействия с выводами процессора (gpio) через файлы в sysfs
 * Считанные и устанавливаемые состояния выводов соответствую физическим состояниям
 * без учета инверсии оптронами на входах
 */
class gpio{
	void 	(* gpio_change_handler_func)(gpio*, void * param);
	void 	*handler_param;
	int		fd;
	uint export_pin;
	bool value;
	bool is_input;
	ulong counter;	//счетчик импульсов на входе
	Timer_ms delay;	//для подавления дребезга при счете
	bool inverted;	//требуется ли инвертировать считанное или устанавливаемое состояние
	ulong cntr_min_pulse_ms;	//минимальная длительность импульса для счета
public:
	gpio(uint8_t exportn, bool invert, bool input, uint8_t set_val,
			void (* change_handler)(gpio *sender, void *param), void *h_param);
	~gpio();
	void update(bool val);
	bool get_val()				{return value;}
	bool get();
	bool set(bool val);
	int get_fd()				{return fd;}
	int get_export_pin()		{return export_pin;}
	ulong get_counter()			{return counter;}
	void set_inversion(bool i)	{inverted = i;}
	bool is_inverted()			{return inverted;}
	void set_counter_min_pulse(ulong pulse_ms)	{cntr_min_pulse_ms = pulse_ms;}
};

//int gpio_export(bool to_output, uint8_t exportn, uint8_t value); //private
//int gpio_set_to_output(uint8_t exportn, uint8_t value);
//int gpio_set_to_input(uint8_t exportn, uint8_t value);

gpio* gpio_add_to_poll_list(uint8_t exportn, bool invert, void (* change_handler)(gpio*, void *param), void *h_param);
gpio* gpio_set_to_output(uint8_t exportn, bool invert, uint8_t value);
//int gpio_set_out_value(uint8_t exportn, uint8_t value);

//bool gpio_set_value(gpio_info_t &gpio, bool value);
//bool gpio_read_value(gpio_info_t &gpio);
//bool gpio_read_value(uint8_t exportn);

timer_p timer_add_timer(uint32_t intval, void (* tmr_handler_func)(void *), void *par);
void timer_set_time(timer_p timer, int val);
void timer_del_timer(timer_p timer);
int poll_add_to_list(int fd, short int events, void (* poll_handler_func)(int fd, short int revents, void *par), void *param = 0);
void poll_delete_from_list(int fd);
void gpio_polling_task();

#endif /* GPIOPOLL_H_ */
