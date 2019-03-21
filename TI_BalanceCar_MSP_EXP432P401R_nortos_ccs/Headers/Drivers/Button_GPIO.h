/*
 * Button_GPIO.h
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */

#ifndef HEADERS_DRIVERS_BUTTON_GPIO_H_
#define HEADERS_DRIVERS_BUTTON_GPIO_H_

#include <./Headers/library.h>
#include <./Headers/Drivers/Motor_Timer_A.h>
#include <./Headers/Users/Control.h>

void Button_init();
void PORT1_IRQHandler(void);

#endif /* HEADERS_DRIVERS_BUTTON_GPIO_H_ */
