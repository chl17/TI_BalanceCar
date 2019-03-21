/*
 * SysTick.h
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */

#ifndef HEADERS_DRIVERS_SYSTICK_H_
#define HEADERS_DRIVERS_SYSTICK_H_

#include <./Headers/library.h>

//Systick tick count, increases 1 per 1/1000 s.
extern long long tick;

void delay_ms(int delay_time);
void SysTick_init();
void SysTick_Handler(void);

#endif /* HEADERS_DRIVERS_SYSTICK_H_ */
