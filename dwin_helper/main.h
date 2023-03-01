/*
 * main.h
 *
 *  Created on: 28 февр. 2023 г.
 *      Author: sergei
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

struct __attribute__((packed)) touch_msg{
	uint16_t 	header;	//5aa5
	uint8_t 	len;
	uint8_t		msg_type;
	uint8_t 	event;
	uint16_t	x;
	uint16_t	y;
	uint8_t		crc;
};

#endif /* MAIN_H_ */
