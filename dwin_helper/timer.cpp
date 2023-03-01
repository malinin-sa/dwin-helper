#include "timer.h"
#include "limits.h"

struct sigevent sig_spec;
int flag_stop = 0;
unsigned long int ms_count = 0;
struct itimerspec timer_setting;
timer_t timer_h;

unsigned long GetTimeTicks() {
	timespec monotime;
	if(clock_gettime(CLOCK_MONOTONIC, &monotime)==0)
		ms_count = monotime.tv_sec * 1000 + monotime.tv_nsec / 1000000;

	return ms_count;
}

//обработка сигнала с таймера
void timer_signal(int sig) {
	ms_count++;
}

void quit_signal(int i) {
	flag_stop = 1;
}
//todo проверить использование SIGUSR1 в station.cpp
//создает таймер и устанавливает ему период сработки в 1 мс
int InitTimer(){
/*

	signal(SIGUSR1, timer_signal);
	signal(SIGINT, quit_signal);

	sig_spec.sigev_notify = SIGEV_SIGNAL;
	sig_spec.sigev_signo = SIGUSR1;

	if (timer_create(CLOCK_REALTIME, &sig_spec, &timer_h) < 0) {
		printf("TIMER: timer_create error\n");
		return 0;
	}

	timer_setting.it_value.tv_sec = 0;
	timer_setting.it_value.tv_nsec = 1000000;
	timer_setting.it_interval.tv_sec = 0;
	timer_setting.it_interval.tv_nsec = 1000000;

	if (timer_settime(timer_h, 0, &timer_setting, NULL) < 0) {
		printf("TIMER: timer_settime error\n");
		return 0;
	}
*/

//	timer_delete(timer_h);

//	printf("MS: %lu\n", ms_count);
	return 1;
}


unsigned long int TimeBetween(unsigned long int Start, unsigned long int Stop)
{
	if (Start <= Stop)
		return Stop - Start;

	return ULONG_MAX - Start + Stop;
}
/*______________________________________________________________________________________________*/
//возвращает время в мс между Start и текущим временем с учетом переполнения
unsigned long int TimeFrom(unsigned long int Start) {
	return TimeBetween(Start, GetTimeTicks());
}

Timer_ms::Timer_ms(void) {
	_timeout = 0;
	_start = GetTimeTicks();
	_Active = false;
	paused = false;
	time_left = 0;
}

Timer_ms::Timer_ms(ulong timeout) {
	Set(timeout);
}

void Timer_ms::Set(ulong timeout) {
//PRINT("\n Timer set = %lu", timeout);
	_timeout = timeout;
	Restart();
}

ulong Timer_ms::TimeElapsed_ms(void) {
	ulong end = GetTimeTicks();
	return ((end >= _start) ? (end - _start) : (ULONG_MAX - _start + end));
}

ulong Timer_ms::TimeLeft() {
	if (paused) return time_left;
	ulong t = TimeElapsed_ms();
	return (_timeout > t) ? (_timeout - t) : 0;
}

void  Timer_ms::Pause(){
	paused = true;
	time_left = TimeLeft();
	_Active = false;
}

void  Timer_ms::Resume(){
	if (paused){
		paused = false;
		Set(time_left);
	}
}

bool Timer_ms::IsOut(void) {
//printf("\n TimeElapsed = %lu, timeout=%lu paused=%i", TimeElapsed_ms(), _timeout, paused);
	if (_timeout == 0)
		return true;
	else if (paused && time_left>0)
		return false;
	return (TimeElapsed_ms() >= _timeout);
}

//------------------------------------------------------------------------------------
DelayOn::DelayOn(ulong delay_ms){
	delay.Set(delay_ms);
}

int DelayOn::Output(int input){
	if (input ==0)
		delay.Restart();
	if (delay.IsOut())
		return 1;
	return 0;
}

//------------------------------------------------------------------------------------
DelayOff::DelayOff(ulong delay_ms) {
	delay.Set(delay_ms);
}

int DelayOff::Output(int input) {
	if (input != 0)
		delay.Restart();
	if (delay.IsOut())
		return 1;
	return 0;
}
