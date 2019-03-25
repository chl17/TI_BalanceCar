/*
 * LED_GPIO.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */
#include <./Headers/Drivers/LED_GPIO.h>

void LED_init(){
    //Red LED on P1.0
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
}
