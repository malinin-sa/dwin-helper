#ifndef _TIMER_
	#define _TIMER_

//#include "..\SYSTEM\lib\7188all.h"
//#include "common.h"
//#include "system.h"
//#include "comdef.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

extern unsigned long int ms_count;
int InitTimer();
unsigned long GetTimeTicks();
unsigned long int TimeFrom(unsigned long int Start);

class Timer_ms
{
	ulong _timeout;
	ulong _start;
	bool _Active;
	bool paused;
	ulong time_left; //хранит оставшееся время в режиме паузы
public:
	Timer_ms(void);
	Timer_ms(ulong timeout);
	ulong GetTimeout() { return _timeout;}
	void Restart()		{_start = GetTimeTicks(); _Active = true;}
	void Set(ulong timeout);
//	void Set(void){ _start = GetTimeTicks(); _Active = 1;}
	ulong TimeElapsed_ms(void);
	float TimeElapsed_s()	{return (float)TimeElapsed_ms()/1000.0;}
	ulong TimeLeft();
	bool IsOut(void);
	bool IsActive(void){return _Active;}
	void Disable(){ _Active = false;}
	void Pause();
	void Resume();
};

///Осуществляет задержку изменения входа из 0 в 1 на заданное время. Изменение из 1 в 0 - без задержки
class DelayOn{
	Timer_ms delay;
public:
	DelayOn()	{};
	DelayOn(ulong delay_ms);
	void SetDelay(ulong delay_ms)	{delay.Set(delay_ms);}
	int Output(int input);
};

///Осуществляет задержку изменения входа из 1 в 0 на заданное время. Изменение из 0 в 1 - без задержки
class DelayOff{
	Timer_ms delay;
public:
	DelayOff()	{};
	DelayOff(ulong delay_ms);
	void SetDelay(ulong delay_ms)	{delay.Set(delay_ms);}
	int Output(int input);
};



#endif


