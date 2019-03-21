/*
 * Gyroscope_UART.h
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */

#ifndef HEADERS_DRIVERS_GYROSCOPE_UART_H_
#define HEADERS_DRIVERS_GYROSCOPE_UART_H_

#include <./Headers/library.h>

//Pitch angle W-omega
extern float Rollx, Pitchy, Yawz, Wx, Wy, Wz;

void UART_init();
/* EUSCI A1 UART ISR  */
void EUSCIA1_IRQHandler(void);

#endif /* HEADERS_DRIVERS_GYROSCOPE_UART_H_ */
