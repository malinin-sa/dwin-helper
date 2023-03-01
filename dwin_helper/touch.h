/*
 * touch.h
 *
 *  Created on: 1 мар. 2023 г.
 *      Author: sergei
 */

#ifndef TOUCH_H_
#define TOUCH_H_

class touch {
	int fd;
	void emit(int fd, int type, int code, int val);
public:
	touch();
	virtual ~touch();

	int get_fd()	{return fd;}
	void event(int x, int y, bool touch);
};

#endif /* TOUCH_H_ */
