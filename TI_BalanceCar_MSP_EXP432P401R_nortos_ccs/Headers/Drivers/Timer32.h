/*
 * Timer32.h
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */

#ifndef HEADERS_DRIVERS_TIMER32_H_
#define HEADERS_DRIVERS_TIMER32_H_

#include <./Headers/library.h>
#include <./Headers/Drivers/Encoder_Timer_A.h>
#include <./Headers/Drivers/Motor_Timer_A.h>
#include <./Headers/Users/Control.h>

void Timer32_init();
void T32_INT1_IRQHandler(void);

#endif /* HEADERS_DRIVERS_TIMER32_H_ */
