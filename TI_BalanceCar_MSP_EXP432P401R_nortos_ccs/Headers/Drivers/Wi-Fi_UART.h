/*
 * Wi-Fi_UART.h
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */

#ifndef HEADERS_DRIVERS_WI_FI_UART_H_
#define HEADERS_DRIVERS_WI_FI_UART_H_

#include <./Headers/library.h>

//Variables in UART IRQHandler to handle data of Wi-fi UART module
extern uint8_t newLineReceived;
extern uint8_t receivedString[100];
extern int receivedStringLength;

void WiFi_init();
void EUSCIA2_IRQHandler(void);
int fputc(int _c, register FILE *_fp);
int fputs(const char *_ptr, register FILE *_fp);

#endif /* HEADERS_DRIVERS_WI_FI_UART_H_ */
