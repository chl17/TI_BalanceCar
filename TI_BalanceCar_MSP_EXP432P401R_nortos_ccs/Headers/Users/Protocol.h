/*
 * Protocol.h
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */

#ifndef HEADERS_USERS_PROTOCOL_H_
#define HEADERS_USERS_PROTOCOL_H_

#include <./Headers/library.h>
#include <./Headers/Drivers/Wi-Fi_UART.h>

#include <./Headers/Users/Functions.h>
#include <./Headers/Users/Control.h>

extern uint8_t protocolString[100];

void Protocol(void);

#endif /* HEADERS_USERS_PROTOCOL_H_ */
