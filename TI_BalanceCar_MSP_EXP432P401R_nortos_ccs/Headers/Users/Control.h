/*
 * Control.h
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */

#ifndef HEADERS_USERS_CONTROL_H_
#define HEADERS_USERS_CONTROL_H_

#include <./Headers/library.h>
#include <./Headers/parameters.h>
#include <./Headers/Drivers/Motor_Timer_A.h>
#include <./Headers/Drivers/Encoder_Timer_A.h>
#include <./Headers/Drivers/Gyroscope_UART.h>

//Start or stop flag
extern int start;
extern float speed;
extern float turnAngle;

void carStop();
int balance();
int velocity();
int turn();

#endif /* HEADERS_USERS_CONTROL_H_ */
