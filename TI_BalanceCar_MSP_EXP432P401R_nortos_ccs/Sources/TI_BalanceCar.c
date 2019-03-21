/*
 * TI_BalanceCar.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */

#include <./Headers/TI_BalanceCar.h>

void TI_BalanceCar_Init(){
    SysTick_init();
    LED_init();
    Motor_init();
    Encoder_init();
    UART_init();
    Button_init();
    LED_init();
    Timer32_init();
    Interrupt_init();
}
