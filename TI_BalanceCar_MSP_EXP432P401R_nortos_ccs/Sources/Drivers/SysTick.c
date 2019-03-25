/*
 * SysTick.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */

#include <./Headers/Drivers/SysTick.h>

long long tick = 0;

//Delay function, delay_time in ms
void delay_ms(int delay_time){
    long long temp = tick;
    while(tick - temp < delay_time);
}

void SysTick_init(){
    /* Configuring SysTick to trigger at 24000 (MCLK is 24MHz so this will
             * make it toggle 1K times every 1s) */

    CS_initClockSignal(CS_MCLK, CS_MODOSC_SELECT, CS_CLOCK_DIVIDER_1);
    SysTick_enableModule();
    SysTick_setPeriod(24000);
    SysTick_enableInterrupt();
}

void SysTick_Handler(void)
{
    tick++;
}
