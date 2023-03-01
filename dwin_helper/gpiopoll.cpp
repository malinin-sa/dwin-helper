/*
 * gpiopoll.c
 *
 *  Created on: 12.11.2013
 *      Author: user
 */
#define TIMER_EXPORTN_VALUE 0xFF

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <pthread.h>
#include <syslog.h>
#include <errno.h>
#include <poll.h>
#include <sys/timerfd.h>
#include "gpiopoll.h"
#include "debug.h"
#include <iostream>
#include <cassert>
#include <stdexcept>
#include <vector>
#include <algorithm>

//#define DEBUG

struct timer_handle_t
{
	struct 	timer_handle_t *next;
	void 	(* timer_handler_func)(void *);
	void 	*param;
	int 	interval;
	int 	left;
};


struct pollfd_t
{
	void (* poll_callback)(int fd, short int revents, void *param);
	int fd;
	short int events;
	void *pParam;	//будет указывать на экземпляр класса
};

//здесь хранятся проинициализированные (экспортированные) gpio
static std::vector <gpio*> gpio_ins;	//входы
static std::vector <gpio*> gpio_outs; //выходы
//static struct gpio_info_t *gpios;
//static uint gpio_count;

static struct timer_handle_t *timer_handles;
static int timer_fd = -1;	//дескриптор таймера в 0,1 сек

static struct pollfd_t *polls;
static uint poll_count;	//количество прочих дескрипторов (не GPIO)

static struct pollfd *pollfds;	//дескрипторы для отслеживания событий
static uint	pollnfds;
static uint8_t	pollfds_uptodate;	//признак необходимости обновить набор дескрипторов

#include <sys/utsname.h>

/*
 * возвращает имя каталога для экспотрируемого gpio с номером gpioid
 * Имя каталога отличается для разных версий ядра
 * Для 3.2.51 gpioid почему-то должен быть больше на 32
 */
int gpioid2sysname(uint gpioid, char *sysname) {
	if (gpioid > 320)
		return 0;
	utsname info;
	uname(&info);
	if (strncmp(info.release, "3.2.51", 5) > 0) {
//		printf("\nkernel version %s", info.release);
		sprintf(sysname, "pio%c%d", 'A' + gpioid / 32, gpioid % 32);
	} else
		sprintf(sysname, "gpio%d", gpioid);
	return 1;
}

//выполняет экспорт пина с номером  exportn в sysfs.
//если input==false тогда пин будет настроен как выход и установлено значение set_val
gpio::gpio(uint8_t exportn, bool invert, bool input, uint8_t set_val,
		void (* change_handler)(gpio *sender, void *param), void *h_param){
	int ret;
	int tmpfd;
	char tmpstr[50];
	int tmpstr_len;
	struct stat info;
	char sysname[10];	//имя директории после экспорта

	is_input = input;
	export_pin = exportn;
	gpio_change_handler_func = change_handler;
	handler_param = h_param;
	inverted = false;
	counter = 0;
	cntr_min_pulse_ms = 100;
	inverted = invert;

	if (gpioid2sysname(exportn, sysname) == 0)
		throw std::runtime_error("GPIO export failed: gpio id is too big");

	// Check if GPIO is already exported
	sprintf(tmpstr, "/sys/class/gpio/%s", sysname);
	if ((stat(tmpstr, &info) != 0)||((info.st_mode & S_IFDIR) == 0))
	{
#ifdef DEBUG
		debug_output(LOG_INFO, 0, "GPIO: %d is not exported, exporting now", exportn);

#endif
		tmpfd = open("/sys/class/gpio/export", O_WRONLY);
		if (tmpfd == -1)
		{
			debug_output(LOG_ERR, 1, "GPIO: cannot open gpio/export file, disabled in kernel?");
			throw std::runtime_error("GPIO export failed:cannot open gpio/export file, disabled in kernel?");
		}

		tmpstr_len = sprintf(tmpstr,"%d",exportn);
		ret = write(tmpfd, tmpstr, tmpstr_len);
		close(tmpfd);
		if (ret != tmpstr_len)
		{
			debug_output(LOG_ERR, 1, "GPIO: write to gpio/export file failed %d of %d bytes written", ret, tmpstr_len);
			throw std::runtime_error("GPIO export failed: write to gpio/export file failed");
		}
	}  // Not exported
	else
	{  // Dir exists - already exported
#ifdef DEBUG
		debug_output(LOG_INFO, 0, "GPIO: %d is already exported", exportn);
#endif
	}

	sprintf(tmpstr,"/sys/class/gpio/%s/direction", sysname);
	tmpfd = open(tmpstr, O_WRONLY);
	if (tmpfd == -1)
	{
		debug_output(LOG_ERR, 1, "GPIO: cannot open %s file", tmpstr);
		throw std::runtime_error("GPIO export failed: cannot open 'direction' file");
	}
	const char* str_dir = input ? "in":"out";
	const int dir_len =  input ? 3:4;
	ret = write(tmpfd, str_dir, dir_len);
	close(tmpfd);
	if (ret != dir_len)
	{
		debug_output(LOG_ERR, 1, "GPIO: write to gpio/direction file failed %d of %d bytes written", ret, dir_len);
		throw std::runtime_error("GPIO export failed: write to gpio/direction file failed");
	}

	sprintf(tmpstr,"/sys/class/gpio/%s/edge", sysname);
	tmpfd = open(tmpstr, O_WRONLY);
	if (tmpfd == -1)
	{
		debug_output(LOG_ERR, 1, "GPIO: cannot open %s file, try to use another gpio?", tmpstr);
		throw std::runtime_error("GPIO export failed: cannot open gpio/edge file");
	}
	ret = write(tmpfd, "both", 5);
	close(tmpfd);
	if (ret != 5)
	{
		debug_output(LOG_ERR, 1, "GPIO: write to gpio/edge file failed %d of 5 bytes written", ret);
		throw std::runtime_error("GPIO export failed: write to gpio/edge file failed");
	}

	sprintf(tmpstr,"/sys/class/gpio/%s/value", sysname);
	tmpfd = open(tmpstr, O_RDWR);
	if (tmpfd == -1)
	{
		debug_output(LOG_ERR, 1, "GPIO: cannot open %s file", tmpstr);
		throw std::runtime_error("GPIO export failed: cannot open gpio/value file");
	}
	fd = tmpfd;

	if(input){
		//сразу же вызовем каллбак-функцию чтобы получить текущие значения
		update(get());
	}else{
		set(set_val);
	}

	//добавляем себя в вектор gpio_xx
	std::vector<gpio*> *pv_gpio = input ? &gpio_ins: &gpio_outs;
	auto n = std::find_if(pv_gpio->begin(), pv_gpio->end(),[&](gpio *elem){
		return elem->export_pin == export_pin;
	});
	if (n== pv_gpio->end())
		pv_gpio->push_back(this);
	else
		throw std::runtime_error("GPIO export failed: gpio already exported?");
	printf("\n GPIO %i created as %s, inversion %s, value at start %i, descriptor %i", exportn, input ? "INPUT":"OUTPUT",
			inverted ? "ON":"OFF",	value, fd);
}

gpio::~gpio()	{
	close(fd);
	std::vector<gpio*> *pv_gpio = is_input ? &gpio_ins: &gpio_outs;
	auto n = std::find_if(pv_gpio->begin(), pv_gpio->end(),[&](gpio *elem){
		return elem->export_pin == export_pin;
	});
	if (n != pv_gpio->end())
		pv_gpio->erase(n);
}

void gpio::update(bool val) {
//	printf("\n gpio::update: pin %i inverted=%i %i -> %i", export_pin, inverted, value, val);
	val = inverted ? !val : val;
//	printf(" -> %i", val);
	if (val != value) {
		value = val;
		if (val)  //обработка счетчика и антидребезг
			delay.Set(cntr_min_pulse_ms);  //по переднему фронту запускаем таймер
		else if (delay.IsOut())
			counter++;	//по заднему, если задержка истекла, увеличиваем счетчик
		if (gpio_change_handler_func)
			gpio_change_handler_func(this, handler_param);
	}
}

//Возвращает физическое значение дискретного выхода или входа
bool gpio::get(){
	char tmpstr[50];
	lseek(fd, 0, SEEK_SET);
	int ret = read(fd, tmpstr, 50);
	if (ret == -1)
	{
		debug_output(LOG_ERR, 1, "GPIO: read from gpio/value file failed: %s",strerror(errno));
		return value; //вернем предудущее значение
	}
	if (ret ==0){
		debug_output(LOG_ERR, 1, "GPIO: read from gpio/value file return 0");
		return value;
	}
	assert(ret >= 0);

	if (tmpstr[0] == '0')	ret = 0;
	else if (tmpstr[0] == '1')	ret = 1;
	else
	{
		debug_output(LOG_WARNING, 0, "GPIO: read from gpio/value is strange: %c", tmpstr[0]);
		ret = 0; // Default
	}
	if (lseek(fd, 0, SEEK_SET) != 0)
		debug_output(LOG_WARNING, 1, "GPIO: lseek() failed");
	return ret;
}


//Устанавливает выход пина gpio. Пин должен быть заранее экспортирован как выход
bool gpio::set(bool val) {
	int ret;
	val = inverted ? !val : val;
	const char *to_write = val ? "1" : "0";
	if (val)
		ret = write(fd, to_write, 2);
	else
		ret = write(fd, to_write, 2);
	assert(ret == 2);
	if (ret != 2) {
		debug_output(LOG_ERR, 1, "GPIO: write to gpio/value file failed", ret);
		return false;
	}
#ifdef DEBUG
	printf("\n GPIO %i set %d", export_pin, val);
#endif
	value=val;
	return true;
}


/* Настраивает GPIO с номером exportn, назначает для него обработчик на изменение change_handler и
 * добавляет его в массив gpios
 * exportn - номер GPIO. Вход
 * cur_val - указатель на переменную в которой возвращается текущее состояние GPIO
 * change_handler - указатель на фукцию, вызываемую при изменении состояния
 * h_param - параметры, передаваемые в change_handler
 */
gpio* gpio_add_to_poll_list(uint8_t exportn, bool invert, void (* change_handler)(gpio*, void *param), void *h_param)
{
	gpio *new_gpio;
	try{
		new_gpio = new gpio(exportn, invert, true, 0, change_handler, h_param);
	}catch (std::runtime_error &err){
		debug_output(LOG_ERR, 0, "%s. GPIO%d", err.what(), exportn);
		return NULL;
	}
	pollfds_uptodate = 0;
#ifdef DEBUG
		debug_output(LOG_INFO, 0, "GPIO: New gpiopoll %d registered", exportn);
#endif
	return new_gpio;
}

//создает gpio, настроенный на выход, без обработчика и устанавливает его в состояние value
//добавляет его в вектор gpio_outs
gpio* gpio_set_to_output(uint8_t exportn, bool invert, uint8_t value) {
	auto n = std::find_if(gpio_outs.begin(), gpio_outs.end(), [&](gpio *elem) {
		return elem->get_export_pin() == exportn;
	});
	if (n == gpio_outs.end()) {
		try {
			gpio_outs.push_back(new gpio(exportn, invert, false, value, NULL, NULL));
		} catch (std::runtime_error &err) {
			debug_output(LOG_ERR, 0, "%s. GPIO%d", err.what(), exportn);
			return NULL;
		}
		return gpio_outs.back();
	}
	return *n;
}

int gpio_set_out_value(uint8_t exportn, uint8_t value) {
	auto n = std::find_if(gpio_outs.begin(), gpio_outs.end(), [&](gpio *elem) {
		return elem->get_export_pin() == exportn;
	});
	if (n != gpio_outs.end()) {
		(*n)->set(value);
		return 0;
	} else {
		printf("/n GPIO %i not found in gpio_outs", exportn);
		return -1;
	}
}

//читает состояние вывода (входа или выхода)
bool gpio_read_value(uint8_t exportn) {
	auto n = std::find_if(gpio_ins.begin(), gpio_ins.end(), [&](gpio *elem) {
		return elem->get_export_pin() == exportn;
	});
	if (n != gpio_ins.end()) {
		return (*n)->get();
	} else {	//ищем в выходах
		auto n = std::find_if(gpio_outs.begin(), gpio_outs.end(), [&](gpio *elem) {
			return elem->get_export_pin() == exportn;
		});
		if (n != gpio_outs.end()) {
			return (*n)->get();
		} else {
			printf("/n GPIO %i not found in exported gpio", exportn);
			throw std::runtime_error(std::string("GPIO ") + std::to_string(exportn) + " not found in gpio_xxx");
			return false;
		}
	}
}
/*

int gpio_set_out_value(uint8_t exportn, uint8_t value)
{
	int tmpfd;
	int ret;
	char * tmpstr;
	char sysname[10];	//имя директории после экспорта

	if (gpioid2sysname(exportn, sysname) == 0)
		return -1;

	tmpstr = (char*)malloc(50);
	if (tmpstr == NULL)
	{
		debug_output(LOG_WARNING, 0, "GPIO: malloc failed");
		return -1;
	}
	sprintf(tmpstr,"/sys/class/gpio/%s/value", sysname);
	tmpfd = open(tmpstr, O_WRONLY);
	if (tmpfd == -1)
	{
		debug_output(LOG_ERR, 1, "GPIO: cannot open %s file", tmpstr);
		free(tmpstr);
		return -1;
	}
	free(tmpstr);
	if (value > 0)
		ret = write(tmpfd, "1", 2);
	else
		ret = write(tmpfd, "0", 2);
	close(tmpfd);
	if (ret != 2)
	{
		debug_output(LOG_ERR, 1, "GPIO: write to gpio/value file failed", ret);
		return -1;
	}
#ifdef DEBUG
//		debug_output(LOG_INFO, 0, "GPIO: %d set out value %d", exportn, value);
#endif
	return 0;
}

int gpio_export(bool to_output, uint8_t exportn, uint8_t value){
	int ret;
	int tmpfd;
	char * tmpstr;
	int tmpstr_len;
	struct stat info;
	char sysname[10];	//имя директории после экспорта

	if (gpioid2sysname(exportn, sysname) == 0)
		return -1;

	tmpstr = (char*)malloc(50);
	if (tmpstr == NULL)
	{
		debug_output(LOG_WARNING, 0, "GPIO: malloc failed");
		return -1;
	}

	sprintf(tmpstr, "/sys/class/gpio/%s", sysname);
	if ((stat(tmpstr, &info) != 0)||((info.st_mode & S_IFDIR) == 0))
	{
#ifdef DEBUG
		debug_output(LOG_INFO, 0, "GPIO: %d is not exported, exporting now", exportn);

#endif
		tmpfd = open("/sys/class/gpio/export", O_WRONLY);
		if (tmpfd == -1)
		{
			debug_output(LOG_ERR, 1, "GPIO: cannot open gpio/export file, disabled in kernel?");
			free(tmpstr);
			return -1;
		}

		tmpstr_len = sprintf(tmpstr,"%d",exportn);
		ret = write(tmpfd, tmpstr, tmpstr_len);
		close(tmpfd);
		if (ret != tmpstr_len)
		{
			debug_output(LOG_ERR, 1, "GPIO: write to gpio/export file failed %d of %d bytes written", ret, tmpstr_len);
			free(tmpstr);
			return -1;
		}
	}  // Not exported
	else
	{  // Dir exists - already exported
#ifdef DEBUG
		debug_output(LOG_INFO, 0, "GPIO: %d is already exported", exportn);
#endif
	}

	sprintf(tmpstr,"/sys/class/gpio/%s/direction", sysname);
	tmpfd = open(tmpstr, O_RDWR);
	if (tmpfd == -1)
	{
		debug_output(LOG_ERR, 1, "GPIO: cannot open %s file", tmpstr);
		free(tmpstr);
		return -1;
	}

	const char *dir = to_output ? "out":"in";
	const uint len = to_output ? 4:3;

	//check direction
	ret = read(tmpfd, tmpstr, 4);
	if (ret == 4 && strstr(tmpstr, dir))
		return 0;	//direction already set as needed


	ret = write(tmpfd, dir, len);
	close(tmpfd);
	free(tmpstr);
	if (ret != len)
	{
		debug_output(LOG_ERR, 1, "GPIO: write to gpio/direction file failed %d of %i bytes written", ret, len);
		return -1;
	}
	//	return 	gpio_set_out_value(exportn, value);
	return 0;
}
*/


/*
//создает gpio, настроенный на вход, без обработчика
//добавляет его в вектор gpio_ins
int gpio_set_to_input(uint8_t exportn, uint8_t value)
{
	auto n = std::find_if(gpio_ins.begin(), gpio_ins.end(), [&](gpio *elem) {
		return elem->get_export_pin() == exportn;
	});
	if (n == gpio_ins.end()) {
		try {
			gpio_ins.push_back(new gpio(exportn, false, 0, NULL, NULL));
		} catch (std::runtime_error &err) {
			debug_output(LOG_ERR, 0, "%s. GPIO%d", err.what(), exportn);
			return -1;
		}
	}
	return 0;
}
*/

/*

//Возвращает физическое значение дискретного выхода или выхода
bool gpio_read_value(uint8_t exportn){
	char tmpstr[50];
	int tmpfd, ret;
	char sysname[10];	//имя директории после экспорта

	if (gpioid2sysname(exportn, sysname) == 0)
		return -1;

	sprintf(tmpstr,"/sys/class/gpio/%s/value", sysname);
	tmpfd = open(tmpstr, O_RDONLY);
	if (tmpfd == -1)
	{
		debug_output(LOG_ERR, 1, "GPIO: cannot open %s file", tmpstr);
		return false;
	}
	ret = read(tmpfd, tmpstr, 50);
	close(tmpfd);
	if (ret == -1)
	{
		debug_output(LOG_ERR, 1, "GPIO: read from gpio/value file failed");
		return false;
	}
//printf("\n GPIO %i value %c", exportn, tmpstr[0]);
	return (tmpstr[0] == '1');
}
*/

/* Настраивает GPIO с номером exportn, назначает для него обработчик на изменение change_handler и
 * добавляет его в массив gpios
 * exportn - номер GPIO. Вход
 * cur_val - указатель на переменную в которой возвращается текущее состояние GPIO
 * change_handler - указатель на фукцию, вызываемую при изменении состояния
 * h_param - параметры, передаваемые в change_handler
 */
/*
int gpio_add_to_poll_list(uint8_t exportn, uint8_t *cur_val, void (* change_handler)(uint8_t exportn, uint8_t value, void *param), void *h_param)
{
	int ret;
	int tmpfd;
	char * tmpstr;
	int tmpstr_len;
	struct stat info;
	char sysname[10];	//имя директории после экспорта

	if (gpioid2sysname(exportn, sysname) == 0)
		return -1;

	tmpstr = (char*)malloc(50);
	if (tmpstr == NULL)
	{
		debug_output(LOG_WARNING, 0, "GPIO: malloc failed");
		return -1;
	}

	// Check if GPIO is already exported
	sprintf(tmpstr, "/sys/class/gpio/%s", sysname);
	if ((stat(tmpstr, &info) != 0)||((info.st_mode & S_IFDIR) == 0))
	{
#ifdef DEBUG
		debug_output(LOG_INFO, 0, "GPIO: %d is not exported, exporting now", exportn);

#endif
		tmpfd = open("/sys/class/gpio/export", O_WRONLY);
		if (tmpfd == -1)
		{
			debug_output(LOG_ERR, 1, "GPIO: cannot open gpio/export file, disabled in kernel?");
			free(tmpstr);
			return -1;
		}

		tmpstr_len = sprintf(tmpstr,"%d",exportn);
		ret = write(tmpfd, tmpstr, tmpstr_len);
		close(tmpfd);
		if (ret != tmpstr_len)
		{
			debug_output(LOG_ERR, 1, "GPIO: write to gpio/export file failed %d of %d bytes written", ret, tmpstr_len);
			free(tmpstr);
			return -1;
		}
	}  // Not exported
	else
	{  // Dir exists - already exported
#ifdef DEBUG
		debug_output(LOG_INFO, 0, "GPIO: %d is already exported", exportn);
#endif
	}

	sprintf(tmpstr,"/sys/class/gpio/%s/direction", sysname);
	tmpfd = open(tmpstr, O_WRONLY);
	if (tmpfd == -1)
	{
		debug_output(LOG_ERR, 1, "GPIO: cannot open %s file", tmpstr);
		free(tmpstr);
		return -1;
	}
	ret = write(tmpfd, "in", 3);
	close(tmpfd);
	if (ret != 3)
	{
		debug_output(LOG_ERR, 1, "GPIO: write to gpio/direction file failed %d of 3 bytes written", ret);
		free(tmpstr);
		return -1;
	}

	sprintf(tmpstr,"/sys/class/gpio/%s/edge", sysname);
	tmpfd = open(tmpstr, O_WRONLY);
	if (tmpfd == -1)
	{
		debug_output(LOG_ERR, 1, "GPIO: cannot open %s file, try to use another gpio?", tmpstr);
		free(tmpstr);
		return -1;
	}
	ret = write(tmpfd, "both", 5);
	close(tmpfd);
	if (ret != 5)
	{
		debug_output(LOG_ERR, 1, "GPIO: write to gpio/edge file failed %d of 5 bytes written", ret);
		free(tmpstr);
		return -1;
	}

	sprintf(tmpstr,"/sys/class/gpio/%s/value", sysname);
	tmpfd = open(tmpstr, O_RDONLY);
	if (tmpfd == -1)
	{
		debug_output(LOG_ERR, 1, "GPIO: cannot open %s file", tmpstr);
		free(tmpstr);
		return -1;
	}

	ret = read(tmpfd, tmpstr, 50);
	if (ret == -1)
	{
		debug_output(LOG_ERR, 1, "GPIO: read from gpio/value file failed");
		close(tmpfd);
		free(tmpstr);
		return -1;
	}

	if (tmpstr[0] == 0x30)	ret = 1;	// Inversion
	else if (tmpstr[0] == 0x31)	ret = 0;
	else
	{
		debug_output(LOG_WARNING, 0, "GPIO: read from gpio/value is strange %c", tmpstr[0]);
		ret = 0; // Default
	}
	if (lseek(tmpfd, 0, SEEK_SET) != 0)
		debug_output(LOG_WARNING, 1, "GPIO: lseek() failed");

	gpios = (gpio_info_t *)realloc(gpios, sizeof(struct gpio_info_t) * (gpio_count + 1));
	if (gpios == NULL)
	{
		debug_output(LOG_WARNING, 0, "GPIO: realloc failed");
		close(tmpfd);
		free(tmpstr);
		return -1;
	}

	gpios[gpio_count].export_pin = exportn;
	gpios[gpio_count].value = ret;
	gpios[gpio_count].gpio_change_handler_func = change_handler;
	gpios[gpio_count].handler_param = h_param;
	gpios[gpio_count].fd = tmpfd;
	pollfds_uptodate = 0;
//	pollfds[gpio_count].events = POLLPRI | POLLERR;
	gpio_count++;
	free(tmpstr);
	if (cur_val != NULL)
		(*cur_val) = ret;
	//сразу же вызовем каллбак-функцию чтобы получить текущие значения
	if (change_handler)
		change_handler(exportn, ret, h_param);
#ifdef DEBUG
		debug_output(LOG_INFO, 0, "GPIO: New gpiopoll %d registered", exportn);
#endif
	return 0;
}
*/

//добавляет новый элемент типа timer_handle_t в связанный список timer_handles
//создает таймер (один для всего списка), который будет срабатывать каждые 0,1 сек.
//intval - уставка времени сек. tmr_handler_func - обработчик
timer_p timer_add_timer(uint32_t intval, void (* tmr_handler_func)(void *), void *par)
{
	struct timer_handle_t *tmp_h, *last;

	tmp_h = (timer_handle_t *)malloc(sizeof(struct timer_handle_t));
	if (tmp_h == NULL)	{
		debug_output(LOG_WARNING, 0, "GPIO: malloc failed");
		return NULL;
	}

	tmp_h->interval = intval;
	tmp_h->left = intval;
	tmp_h->timer_handler_func = tmr_handler_func;
	tmp_h->param = par;
	tmp_h->next = NULL;

	if (timer_handles == NULL)	{
		timer_handles = tmp_h;
		timer_fd = timerfd_create(CLOCK_MONOTONIC, 0);
		if (timer_fd < 0)
		{
			free(tmp_h);
			timer_handles = NULL;
			debug_output(LOG_ERR, 1, "GPIO: timerfd_create() failed");
			return NULL;
		}
		{
			struct itimerspec tspec;
			tspec.it_value.tv_sec = 0;			//время до первой сработки таймера
			tspec.it_value.tv_nsec = 100000000L; // 100ms
			tspec.it_interval.tv_sec = 0;		//время между периодическими сработками
			tspec.it_interval.tv_nsec = 100000000L;  // 100ms
			timerfd_settime(timer_fd, 0, &tspec, NULL);
		}
		pollfds_uptodate = 0;
	}
	else {
		// Already have some timers
		last = timer_handles;
		while (last->next != NULL)
			last = last->next;

		last->next = tmp_h;
	}
#ifdef DEBUG
		debug_output(LOG_INFO, 0, "TIMER: New timer registered with interval %d. func ptr = %p", intval, tmr_handler_func);
#endif

	return tmp_h;
}

//устанавливает новый интервал таймера на 0,1*val sec
void timer_set_time(timer_p timer, int val){
	struct timer_handle_t* tmp = static_cast<timer_handle_t*>(timer);
	tmp->interval = val*10;
	tmp->left = val*10;
}

void timer_del_timer(timer_p timer)
{
	struct timer_handle_t *cur;
	if (timer == NULL) return;
	if (timer == timer_handles)
	{
		timer_handles = timer_handles->next;
		free(timer);
		if (timer_handles == NULL)
		{
			close(timer_fd);
			timer_fd = -1;
			pollfds_uptodate = 0;
		}
	}
	else
	{
		cur = timer_handles;
		while ((cur != NULL)&&(cur->next != timer))
			cur = cur->next;
		if (cur != NULL) {
			cur->next = ((struct timer_handle_t *) timer)->next;
			free(timer);
		}
	}
}

//добавляет новый элемент в список polls
int poll_add_to_list(int fd, short int events, void (* poll_handler_func)(int fd, short int revents, void *par), void *param)
{
	if (fd < 0) return -1;
	polls = (pollfd_t *)realloc(polls, sizeof(struct pollfd_t) * (poll_count + 1));
	if (polls == NULL)
	{
		debug_output(LOG_ERR, 0, "POLL: realloc failed");
		return -1;
	}

	polls[poll_count].fd = fd;
	polls[poll_count].events = events;
	polls[poll_count].poll_callback = poll_handler_func;
	polls[poll_count].pParam = param;
	poll_count++;
	pollfds_uptodate = 0;
	return 0;
}

void poll_delete_from_list(int fd)
{
	uint32_t i = 0;
	if (fd < 0) return;
#ifdef	DEBUG
	debug_output(LOG_DEBUG, 0, "POLL: deleting fd: %d from poll list", fd);
#endif
	while ((i < poll_count)&&(polls[i].fd != fd)) i++;
	if ((i < poll_count)&&(polls[i].fd == fd))
	{
		poll_count--;
		pollfds_uptodate = 0;
//		if (i != poll_count - 1)
			memmove(&polls[i], &polls[i+1], sizeof(struct pollfd_t) * (poll_count - i));
		polls = (pollfd_t *)realloc(polls, sizeof(struct pollfd_t) * (poll_count));
		if ((polls == NULL)&&(poll_count > 0))
			debug_output(LOG_ERR, 0, "POLL: realloc failed");
	}
}


void gpio_polling_task()
{
	uint i;
	int ret;
	union {
		uint8_t i_8[8];
		uint64_t i_64;
	} input;
	struct timer_handle_t *cur;
	if (!pollfds_uptodate)
	{	// pollfds invalidated, let's create new
		free(pollfds);
		pollnfds = gpio_ins.size() + poll_count;
		if (timer_handles != NULL)
			pollnfds++;
		pollfds = (pollfd *)malloc(sizeof(struct pollfd) * (pollnfds));
		if ((pollfds == NULL)&&(pollnfds > 0))
		{
			debug_output(LOG_ERR, 0, "POLL_TASK: malloc failed");
			return;
		}
		for (i = 0; i < gpio_ins.size(); i++)
		{
			pollfds[i].fd = gpio_ins[i]->get_fd();
			pollfds[i].events = POLLPRI | POLLERR;
			pollfds[i].revents = 0;
		}
		for (i = 0; i < poll_count; i++)
		{
			pollfds[i + gpio_ins.size()].fd = polls[i].fd;
			pollfds[i + gpio_ins.size()].events = polls[i].events;
			pollfds[i + gpio_ins.size()].revents = 0;
		}
		if (timer_handles != NULL)
		{
			pollfds[poll_count + gpio_ins.size()].fd = timer_fd;
			pollfds[poll_count + gpio_ins.size()].events = POLLIN;
			pollfds[poll_count + gpio_ins.size()].revents = 0;
		}
		pollfds_uptodate = 1;
	}

//проверим, что произошло
	ret = poll(pollfds, pollnfds, -1);
	if (ret < 0)
	{
		if (errno != EINTR)
			debug_output(LOG_WARNING, 1, "GPIO: poll() failed");
	}
	else if (ret == 0)
		debug_output(LOG_INFO, 0, "GPIO: poll() returned zero");
	else
	{
		for (i = 0; i < pollnfds; i++)
		{
			if (pollfds[i].revents > 0)  // is that ok?
			{
				if ((pollfds[i].fd == timer_fd)||(i < gpio_ins.size()))
				{	//читаем по дескриптору 8 байт в input
					if (read(pollfds[i].fd, input.i_8, 8) <= 0)
						debug_output(LOG_ERR, 1, "GPIO: read from gpio/value file failed");
					else
					{	//если дескриптор принадлежит 0,1 сек таймеру
						if (pollfds[i].fd == timer_fd)
						{	// Timer expired
							cur = timer_handles;
							//уменьшаем счетчик у таймеров в списке. Если он обнулился - вызываем обработчик
							while (cur != NULL)
							{
								cur->left -= /**(uint64_t*)*/ input.i_64;
								if (cur->left <= 0)
								{
									if (cur->timer_handler_func){
										cur->timer_handler_func(cur->param);
									}
									cur->left = cur->interval;
								}
								cur = cur->next;
							}
						}
						else
						{	// GPIO changed
							//если значение пина поменялось - вызываем обработчик
#ifdef DEBUG
							debug_output(LOG_INFO, 0, "GPIO: value changed on gpio: %d, value %i",
									gpio_ins[i]->get_export_pin(), (input.i_8[0]=='1') ^ gpio_ins[i]->is_inverted() );
#endif
							switch (input.i_8[0])
							{
							case 0x30:
								gpio_ins[i]->update(/*gpio_ins[i]->is_inverted() ? true : */false);
								/*
								if (gpios[i].value == 0)
								{
									gpios[i].value = 1;	// Inversion
									if (gpios[i].gpio_change_handler_func)
										gpio_ins[i].gpio_change_handler_func(gpio_ins[i].export_pin, gpios[i].value, gpios[i].handler_param);
								}*/
								break;
							case 0x31:
								gpio_ins[i]->update(/*gpio_ins[i]->is_inverted() ? false:*/true);
								/*
								if (gpios[i].value == 1)
								{
									gpios[i].value = 0;	// Inversion
									if (gpios[i].gpio_change_handler_func)
										gpio_ins[i].gpio_change_handler_func(gpios[i].export_pin, gpios[i].value, gpios[i].handler_param);
								}*/
								break;
							default:
								debug_output(LOG_WARNING, 0, "GPIO: read from gpio/value is strange %c", input);
//								gpio_ins[i].value = 0; // Default
								break;
							}//вернем позицию чтения/записи на начала файла
							if (lseek(pollfds[i].fd, 0, SEEK_SET) != 0)
								debug_output(LOG_WARNING, 1, "GPIO: lseek() failed");
						}  // GPIO Changed
					}  // read() succeeded
				}  // Timer or GPIO event
				else	//события на прочих ... штуках (ком порты)
				{	// Generic fd poll event
#ifdef DEBUG
					if (polls[i - gpio_ins.size()].fd != pollfds[i].fd)
						debug_output(LOG_ERR, 0, "POLL: this should not happen");
#endif
					// Event handler is responsible for reading data, etc.
					if (polls[i - gpio_ins.size()].poll_callback)
						polls[i - gpio_ins.size()].poll_callback(pollfds[i].fd, pollfds[i].revents, polls[i - gpio_ins.size()].pParam);
				}	// Other fd poll event
			}  // Have events in current pollfds[i].revents
			pollfds[i].revents = 0;
		} // for() cycle
	} // poll() succeeded
}

