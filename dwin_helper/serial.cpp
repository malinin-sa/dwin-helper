/*
 * serial.cpp
 *
 *  Created on: 1 мар. 2023 г.
 *      Author: sergei
 */

#include "serial.h"
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <exception>
#include <stdexcept>
#include "debug.h"
#include "gpiopoll.h"
#include "time.h"
#include "timer.h"
#include <sys/timerfd.h>
#include <algorithm>

#define DEFAULT_COMPORT_BUFFER_LENGTH 50

const char * strSerialStatus[]={
		"порт свободен",	//порт свободен, не удалось записать в порт
		"Запрос отправлен. Ожидание",	//запрос отправлен, ждем ответ
		"Прием ответа",	//приходят данные
		"Ошибка"
};

serial::serial(const char * device, int speed, char parity, bool one_stop, bool is485, int timeout, int chartime){
	//	struct termios trcfg;

		debug_output(LOG_DEBUG, 0, "serial: port %s speed=%i format=8%c%i", device, speed, parity, one_stop ? 1 : 2);

		if ((timeout <= 0)||(chartime <= 0))
			throw invalid_argument("timeout and chartime must be more than zero");

		port_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL | O_NONBLOCK);  // |O_NDELAY
		if (port_fd < 0) {
			debug_output(LOG_ERR, 1, "serial: failed to open device %s", device);
			throw invalid_argument("can not open port");
		}

		setserial(port_fd, speed, 8, parity, one_stop);
		rs485_mode(port_fd, is485);

		buffer = (uint8_t*) malloc(DEFAULT_COMPORT_BUFFER_LENGTH);
		if (buffer == NULL) {
			debug_output(LOG_ERR, 0, "serial: failed to allocate memory for a buffer");
			throw bad_alloc();
		}
		buf_size = DEFAULT_COMPORT_BUFFER_LENGTH;

		timer_fd = timerfd_create(CLOCK_MONOTONIC, O_NONBLOCK);
		if (timer_fd < 0) {
			debug_output(LOG_ERR, 1, "serial: timerfd_create() failed");
			exit(1);
		}

		if (poll_add_to_list(timer_fd, POLLIN, serial::port_timerfd_poll_handler, this)	!= 0) {
			debug_output(LOG_ERR, 0, "serial: failed to add timerfd to poll list");
			exit(1);
		}
		if (poll_add_to_list(port_fd, POLLIN, serial::port_poll_handler, this) != 0)	{
			debug_output(LOG_ERR, 0, "serial: failed to add serial fd to poll list");
			exit(1);
		}

		state = STATE_IDLE;
		cur_client = 0;
		datalen = 0;
		cmd_timeout = timeout;
		char_timeout = chartime;
		poll_cycle_time_ms = 100;
		debug = false;
		dev_name = device;
		set_1sec_interval();

		int bits = 1 + 8 + (parity == 'n' ? 0 : 1) + (one_stop ? 1 : 2);
		one_byte_time = (1e6 * bits) / speed;

		debug_output(LOG_DEBUG, 0, "serial: %s created, one_byte %i mks", device, one_byte_time);
	}

	serial::~serial(){
		poll_delete_from_list(port_fd);
		poll_delete_from_list(timer_fd);
		close(port_fd);
		close(timer_fd);
		free(buffer);
	}


int setserial(int s, int speed,int data,unsigned char parity, bool one_stopb){
	struct termios tcfg;
	struct termios *cfg = &tcfg;
	tcgetattr(s, cfg);
	cfmakeraw(cfg);
	switch(speed){
		case 50     : { cfsetispeed(cfg,B50)    ; cfsetospeed(cfg,B50)    ; break; }
		case 75     : { cfsetispeed(cfg,B75)    ; cfsetospeed(cfg,B75)    ; break; }
		case 110    : { cfsetispeed(cfg,B110)   ; cfsetospeed(cfg,B110)   ; break; }
		case 134    : { cfsetispeed(cfg,B134)   ; cfsetospeed(cfg,B134)   ; break; }
		case 150    : { cfsetispeed(cfg,B150)   ; cfsetospeed(cfg,B150)   ; break; }
		case 200    : { cfsetispeed(cfg,B200)   ; cfsetospeed(cfg,B200)   ; break; }
		case 300    : { cfsetispeed(cfg,B300)   ; cfsetospeed(cfg,B300)   ; break; }
		case 600    : { cfsetispeed(cfg,B600)   ; cfsetospeed(cfg,B600)   ; break; }
		case 1200   : { cfsetispeed(cfg,B1200)  ; cfsetospeed(cfg,B1200)  ; break; }
		case 1800   : { cfsetispeed(cfg,B1800)  ; cfsetospeed(cfg,B1800)  ; break; }
		case 2400   : { cfsetispeed(cfg,B2400)  ; cfsetospeed(cfg,B2400)  ; break; }
		case 4800   : { cfsetispeed(cfg,B4800)  ; cfsetospeed(cfg,B4800)  ; break; }
		case 9600   : { cfsetispeed(cfg,B9600)  ; cfsetospeed(cfg,B9600)  ; break; }
		case 19200  : { cfsetispeed(cfg,B19200) ; cfsetospeed(cfg,B19200) ; break; }
		case 38400  : { cfsetispeed(cfg,B38400) ; cfsetospeed(cfg,B38400) ; break; }
		case 57600  : { cfsetispeed(cfg,B57600) ; cfsetospeed(cfg,B57600) ; break; }
		case 115200 : { cfsetispeed(cfg,B115200); cfsetospeed(cfg,B115200); break; }
		case 230400 : { cfsetispeed(cfg,B230400); cfsetospeed(cfg,B230400); break; }
	}
	switch(parity/*|32*/){
		case 'n' : { cfg->c_cflag &= ~PARENB; cfg->c_iflag &= ~INPCK; break; }
		case 'e' : { cfg->c_cflag |= PARENB; cfg->c_cflag &= ~PARODD; cfg->c_iflag |= INPCK; break; }
		case 'o' : { cfg->c_cflag |= PARENB; cfg->c_cflag |= PARODD ; cfg->c_iflag |= INPCK; break; }
	}
	cfg->c_cflag &= ~CSIZE;
	switch(data){
		case 5 : { cfg->c_cflag |= CS5; break; }
		case 6 : { cfg->c_cflag |= CS6; break; }
		case 7 : { cfg->c_cflag |= CS7; break; }
		case 8 : { cfg->c_cflag |= CS8; break; }
	}
	if(one_stopb)cfg->c_cflag&=~CSTOPB;
	else cfg->c_cflag|=CSTOPB;

	cfg->c_cflag |= (CREAD | CLOCAL);
	cfg->c_cflag  &= ~CRTSCTS; //disable hardware flow control
	cfg->c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	/* software flow control disabled */
	cfg->c_iflag &= ~(IXON | IXOFF | IXANY);
	/* raw output */
	cfg->c_oflag &= ~OPOST;
	cfg->c_cc[VMIN] = 0;
	cfg->c_cc[VTIME] = 0;

	return tcsetattr(s,TCSANOW,cfg);
}

void rs485_mode(int fd, int on) {
	struct serial_rs485 rs485conf;

	ioctl(fd, TIOCGRS485, &rs485conf);

	if (on) {
		rs485conf.flags |= SER_RS485_ENABLED;
		rs485conf.delay_rts_before_send = 0x00000004;
		ioctl(fd, TIOCSRS485, &rs485conf);
	} else {
		rs485conf.flags &= ~SER_RS485_ENABLED;
		ioctl(fd, TIOCSRS485, &rs485conf);
	}
}

//каллбак функция на получение новых данных
void serial::port_poll_handler(int fd, short int revents, void *param){
	if (param)
		(static_cast<serial*>(param))->read_port();
}

//каллбак функция на таймер
void serial::port_timerfd_poll_handler(int fd, short int revents, void *param){
	if (param)
		(static_cast<serial*>(param))->timer_tick();
}

void serial::set_timer_mks(ulong timeout_mks){
	struct itimerspec tspec;
	tspec.it_value.tv_sec = timeout_mks/1000000;
	tspec.it_value.tv_nsec = (timeout_mks % 1000000) * 1000L;
	tspec.it_interval.tv_sec = timeout_mks/1000000;
	tspec.it_interval.tv_nsec = (timeout_mks % 1000000) * 1000L;
	timerfd_settime(timer_fd, 0, &tspec, NULL);
	if (debug) printf("\n set_timer_mks %lu mks. time %lu", timeout_mks, GetTimeTicks());
}


void serial::set_char_interval_mks(ulong timeout_mks){
	set_timer_mks(timeout_mks);
//	if (debug) printf("\n set_char_interval %lu mks. time %lu", timeout_mks, GetTimeTicks());
}

void serial::set_char_interval(){
	set_timer_mks(char_timeout*1000);
//	if (debug) printf("\n set_char_interval %i ms", char_timeout);
}

//задает таймаут ответа и таймаут между байтами. Оба таймаута должны быть ненулевыми. Иначе таймер
//либо не запустится, либо отработает только задержку it_value
void serial::set_std_interval(){
	set_timer_mks(cmd_timeout*1000);
//	if (debug) printf("\n set_std_interval %i ms now %lu", cmd_timeout, GetTimeTicks());
}

void serial::set_1sec_interval(){
	set_timer_mks(1e6);
}

//переходим к следующему в очереди клиенту
void serial::next_client(){
	if (clients.size() == 0)
		return;
	if (++cur_client == clients.size())
		cur_client = 0;
//	printf("\n serial::next_client: cur_client=%i total=%i", cur_client, clients.size());
	cmd_timeout = clients[cur_client]->get_cmd_timeout();
//	char_timeout = clients[cur_client]->get_char_timeout();
}

bool serial::add_client(serial_client *client){
	if (client == NULL)
		return false;
	if (find(clients.begin(), clients.end(), client)== clients.end()){
		clients.push_back(client);
//		printf("\nserial: client added. total clients %i", clients.size());
		return true;
	}
	debug_output(LOG_WARNING, 0, "serial: try to add already existing client");
	return false;
}

void serial::remove_client(serial_client *client){
	clients.erase(remove(clients.begin(), clients.end(), client), clients.end());
}

//Читает из порта с дескриптором fd. если считалось нормально, ставит статус MSGSTATE_RCV
int serial::read_port(){
	if (debug) printf("\nserial: read_port: %lu ms ",  GetTimeTicks());

	if (state == STATE_IDLE){
		if (debug) debug_output(LOG_WARNING, 0, "serial: receive begin");
		datalen = 0;
	}
	state = STATE_RCV;
	int ret = read(port_fd, &(buffer[datalen]), buf_size - datalen);
	if (ret < 0) {
		debug_output(LOG_ERR, 1, "serial: read from serial port failed");
		return -1;
	}

	if(debug){
		//	addtime();
		//	debug_output(LOG_DEBUG, 0, "%s serial: read %d bytes from addr %d",timestr, ret, buffer[0]);
		printf("\nserial: read %d bytes: %.*s\n", ret, ret, &(buffer[datalen]));
		for (uint i=0; i<datalen; i++)
			printf(" 0x%02X", buffer[i]);
	}
	datalen += ret;
	if (datalen >= buf_size) {
		debug_output(LOG_ERR, 0, "serial (portfd): input buffer overflow");
		tcflush(port_fd, TCIFLUSH);
	} else {
		//запрашиваем у клиента, какой длины должен быть ответ
		uint len = clients[cur_client]->estimated_resp_size(buffer, datalen);
		if(debug) printf("\nserial: estimated response len=%i (%i left)", len, len-datalen);
		if (len > 0) {
			if (datalen >= len) {
				state = STATE_SENT;
				timer_tick();
			}else //если длина ответа известна, она определяет таймаут
				set_char_interval_mks(((len-datalen)*one_byte_time*1.2));
		}else//если нет - таймаут определяется настройкой порта
			set_char_interval();
	}
	return ret;
}


//обработка таймера. Либо истек таймаут на конец фрейма, либо таймаут на ответ слэйва
void serial::timer_tick(){
	uint64_t	expirations;
	uint32_t	buflen=0;
	int 		ret;

/*	if(debug){
		//	addtime();
		printf("\nserial: timer_tick: %lu ms ",  GetTimeTicks());
		itimerspec timeouts;
		timerfd_gettime(timer_fd, &timeouts);
		printf("timer settings interval %lu left %lu ms", timeouts.it_interval.tv_nsec/1000000, timeouts.it_value.tv_nsec/1000000);
		printf(" state %i - %s", state, strSerialStatus[state]);
	}*/
	//прочитаем, сколько раз истек таймер, чтобы обнулить признак готовности
	if (read(timer_fd, &expirations, sizeof(expirations)) <= 0)
//		if(debug) debug_output(LOG_ERR, 1, "serial: read from timerfd failed");
	if(debug) printf("\n timer expired %llu times", expirations);

	if (clients.size() == 0) {
		set_1sec_interval();
		return;
	}

	switch (state)	{
	case STATE_RCV:
		// Normal condition, check if there is another pack of data in port buffer
		ret = read_port();
		if (ret < 0) return;
		// Have new data, let's wait some time, maybe more will come
		if (ret > 0 && datalen < buf_size){
#ifdef	_MBMASTER_DEBUG_L5
			debug_output(LOG_DEBUG, 0, "serial: read additional %i bytes", ret);
#endif
			return;
		}
		//сюда попадаем если чтение порта ничего не вернуло
		//считаем, что прием фрейма закончен, отдаем его клиенту
	case STATE_SENT:
		// No data since transmission, this is a timeout condition, buflen will be zero

		if(debug){
			//		addtime();
			//		printf("\n%s MBus_Master: transaction complete, total recived %d bytes",timestr, crport->buflen);
			printf("\n serial: transaction complete, total recived %d bytes. Time %lu ms", datalen,  GetTimeTicks());
			if (datalen){
				printf("\nReceived: %.*s \n", datalen, buffer);
				for (uint i = 0; i < datalen; i++)
					printf("%02X ", buffer[i]);
			}
		}

		clients[cur_client]->result(buffer, datalen);
		// increase iteration (and next client if needed)
		next_client();
		//установим таймер на 100 мс чтобы следующий запрос был с задержкой
		set_char_interval_mks(poll_cycle_time_ms * 1000UL);
		state = STATE_IDLE;
		return;
		//	mbus_inc_iteration(crport);
		/* no break */
		// Fallinto next

		//порт свободен - получаем от клиента новый запрос
	case STATE_IDLE:
		//проверка таймаута опроса клиента
		if (clients[cur_client]->isReadyToPoll())
			buflen = clients[cur_client]->request(0, buffer, buf_size);
		//		buflen = clients[cur_client]->request(0, buffer, buf_size);
		int i=0;
		while(buflen == 0 && i < clients.size()) {
			next_client();
			i++;
			if (clients[cur_client]->isReadyToPoll())
				buflen = clients[cur_client]->request(0, buffer, buf_size);
		}
		//клиенты ничего не хотят передавать
		if(buflen == 0){
			state = STATE_IDLE;
			return;
		}

		tcflush(port_fd, TCIOFLUSH);
		if(debug){
			//		addtime();
			//			debug_output(LOG_DEBUG, 0, "%s MBus_Master: write %d bytes to serial port", timestr, buflen);
			printf("\n\nserial: %lu ms ",  GetTimeTicks());
			printf("\nserial: write %d bytes to %s: %.*s", buflen, dev_name.c_str(), buflen, buffer);
			printf("\nserial Sending: ");
			for (int i = 0; i < buflen; i++)
				printf("%02X ", buffer[i]);
		}
		ret = write(port_fd, buffer, buflen);
		clients[cur_client]->UpdatePollTimestamp();
		if (ret == -1)
		{
			debug_output(LOG_WARNING, 1, "serial: write to serial port failed");
			// Maybe best solution is to close port?
			clients[cur_client]->result(buffer, 0);
			state = STATE_IDLE;
		}
		else
		{	// Message sent, now let's wait for response
			//попробуем в конце таймаута приема что-нибудь прочитать, даже если не было событий по порту
			state = clients[cur_client]->need_response ? STATE_RCV : STATE_IDLE;
			datalen = 0;
		}
		set_std_interval();
		break;

	} // switch
}


//Признак готовности клиента к опросу
//Если устройство на связи - готовность определяется периодом опроса
//Если не на связи - количеством пропущенных циклов опроса
bool serial_client::isReadyToPoll() {
//	printf("\n isReadyToPoll: %s", online ? "online":"Offline");
	if (online) {
		offline_try_cntr = 0;
//		if(poll_delay_ms) printf("\n IsReadyToPoll: time left %lu", poll_delay_ms- TimeFrom(last_poll_timestamp));
		return (poll_delay_ms == 0 || TimeFrom(last_poll_timestamp) > poll_delay_ms);
	} else if (offline_try_cntr < offline_try_after) {
		//если нет связи и запросов пропущено еще недостаточно
		offline_try_cntr++;
		return false;  // пропускаем этот запрос
	}
	return true;
}

serial_client::serial_client() {
	link_nacks = 0;
	max_noacks = 5;
	online = true; //для реакции, если при старте программы устройство не отвечает
	last_poll_timestamp = GetTimeTicks();
	poll_delay_ms = 0;
	cmd_timeout = 200;
	char_timeout = 20;
	offline_try_after = 5;
	offline_try_cntr = 0;
	need_response = true;
}





