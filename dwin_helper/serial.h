/*
 * serial.h
 *
 *  Created on: 1 мар. 2023 г.
 *      Author: sergei
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <vector>
#include "stdint.h"
#include "sys/types.h"
#include <string>
#include "timer.h"

using namespace std;

enum PORTSTATE{
		STATE_IDLE,	//порт свободен, не удалось записать в порт
		STATE_SENT,	//запрос отправлен, ждем ответ
		STATE_RCV,	//приходят данные
		STATE_ERR,
//		STATE_WAIT	//ожидание
};

int setserial(int s, int speed, int data, unsigned char parity, bool one_stopb);
void rs485_mode(int fd, int on) ;

class serial_client{
	uint 		offline_try_cntr; //счетчик пропускаемых запросов в offline
protected:
	uint link_nacks; //счетчик запросов без ответа
	uint max_noacks; //количество запросов без ответа до перехода в offline
	bool online;
	uint16_t	cmd_timeout;
	uint16_t	char_timeout;
//	char		endchar; //символ конца пакета
	uint 		offline_try_after;	//сколько запросов пропускать в offline перед след. попыткой
	uint		poll_delay_ms;	//период опроса. 0-опрос на каждом цикле
	ulong		last_poll_timestamp;	//Время последнего запроса
public:
	serial_client()	;
	virtual ~serial_client()	{};
	// формирует запрос. возвращает длину запроса
	virtual int request(int retry, uint8_t *buffer, uint bufsize)=0;
	// разбор ответа
	virtual void result (uint8_t *buffer, uint length)	=0;
	bool IsOnLine()	{return online;}
	void SetOnline()	{online = true;} //используется для сброса аварии
	uint16_t get_cmd_timeout()	{return cmd_timeout;}
	uint16_t get_char_timeout()	{return char_timeout;}
//	char get_endchar()			{return endchar;}
	bool		need_response;	//нужно ли ждать ответ на запрос

	void set_poll_delay(uint delay_ms)	{poll_delay_ms = delay_ms;}
	bool isReadyToPoll();
	void UpdatePollTimestamp()	{last_poll_timestamp = GetTimeTicks();} //обновить время последнего опроса
	virtual uint estimated_resp_size(uint8_t *buffer, uint length)	{return 0;}
};

class serial {
//	struct termios old_stdio;
	int 		port_fd;
	int			timer_fd;
	uint8_t		*buffer;
	unsigned	buf_size;
	unsigned	datalen;
	vector<serial_client*> clients;
	int			cur_client;
	PORTSTATE	state;
	string dev_name;	//имя порта
	uint16_t	cmd_timeout; //таймаут ответа прибора
	uint16_t	char_timeout; //таймаут конца ответа
	uint16_t	one_byte_time; //время передачи одного байта (mks)
	unsigned	poll_cycle_time_ms;	//период опроса

	int read_port();
	void timer_tick();
	void next_client();
public:
	bool debug;

	serial(const char * device, int speed, char parity, bool one_stop, bool is485, int timeout, int chartime);
	virtual ~serial();
	static void port_poll_handler(int fd, short int revents, void *param);
	static void port_timerfd_poll_handler(int fd, short int revents, void *param);

	void set_poll_time(uint poll_time)	{poll_cycle_time_ms = poll_time;}
	void set_timer_mks(ulong timeout_mks);
	void set_char_interval();
	void set_char_interval_mks(ulong	timeout_mks);
	void set_std_interval();
	void set_1sec_interval();
	bool add_client(serial_client *client);
	void remove_client(serial_client *client);
};

#endif /* SERIAL_H_ */
