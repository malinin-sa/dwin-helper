/*
 * touch.cpp
 *
 *  Created on: 1 мар. 2023 г.
 *      Author: sergei
 */

#include "touch.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <linux/uinput.h>

//create virtual input device
touch::touch() {
	struct uinput_setup usetup;
	struct uinput_abs_setup abssetup;
	fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);

	/* enable mouse button left and relative events */
	ioctl(fd, UI_SET_EVBIT, EV_KEY);
	ioctl(fd, UI_SET_KEYBIT, BTN_LEFT);

	ioctl(fd, UI_SET_EVBIT, EV_ABS);
	ioctl(fd, UI_SET_ABSBIT, ABS_X);
	ioctl(fd, UI_SET_ABSBIT, ABS_Y);

	memset(&usetup, 0, sizeof(usetup));
	usetup.id.bustype = BUS_USB;
	usetup.id.vendor = 0x1234; /* sample vendor */
	usetup.id.product = 0x5678; /* sample product */
	strcpy(usetup.name, "DWIN touchscreen");

	ioctl(fd, UI_DEV_SETUP, &usetup);

	abssetup.code = ABS_X;
	abssetup.absinfo.minimum = 0;
	abssetup.absinfo.maximum = 1024;
	abssetup.absinfo.resolution = 1;
	abssetup.absinfo.value = 0;
	abssetup.absinfo.flat = 2;
	abssetup.absinfo.fuzz = 2;
	ioctl(fd, UI_ABS_SETUP, &abssetup);

	abssetup.code = ABS_Y;
	abssetup.absinfo.minimum = 0;
	abssetup.absinfo.maximum = 600;
	abssetup.absinfo.resolution = 1;
	abssetup.absinfo.value = 0;
	abssetup.absinfo.flat = 2;
	abssetup.absinfo.fuzz = 2;
	ioctl(fd, UI_ABS_SETUP, &abssetup);

	ioctl(fd, UI_DEV_CREATE);
}

touch::~touch() {
	ioctl(fd, UI_DEV_DESTROY);
	close(fd);
}

void touch::emit(int fd, int type, int code, int val) {
	struct input_event ie;

	ie.type = type;
	ie.code = code;
	ie.value = val;
	/* timestamp values below are ignored */
	ie.time.tv_sec = 0;
	ie.time.tv_usec = 0;

	write(fd, &ie, sizeof(ie));
}

void touch::event(int x, int y, bool touch) {
	printf("\n touch_event x=%i y=%i touch=%i", x,y,touch);
	emit(fd, EV_ABS, ABS_X, x);
	emit(fd, EV_ABS, ABS_Y, y);
	emit(fd, EV_KEY, BTN_LEFT, touch);
	emit(fd, EV_SYN, SYN_REPORT, 0);
}
