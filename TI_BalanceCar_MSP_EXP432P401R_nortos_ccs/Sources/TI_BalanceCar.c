/*
 * TI_BalanceCar.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */

#include <./Headers/TI_BalanceCar.h>

//Initialize all modules
void TI_BalanceCar_Init(){
    SysTick_init();
    LED_init();
    Motor_init();
    Encoder_init();
    Gyroscope_init();
    WiFi_init();
    Button_init();
    Timer32_init();
    Interrupt_init();
}
