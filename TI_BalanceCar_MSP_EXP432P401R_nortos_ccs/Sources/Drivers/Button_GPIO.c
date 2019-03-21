/*
 * Button_GPIO.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */
#include <./Headers/Drivers/Button_GPIO.h>

void Button_init(){
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);

    /* Enabling interrupts */
    MAP_Interrupt_enableInterrupt(INT_PORT1);
}

/* Port1 ISR - This ISR will progressively step up the duty cycle of the PWM(P1.1) and toggle start flag(P1.4)
 * on a button press
 */
void PORT1_IRQHandler(void)
{
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if (status & GPIO_PIN1)
    {
        if(pwmConfigLeft.dutyCycle == 3000){
            pwmConfigLeft.dutyCycle = 0;
            pwmConfigRight.dutyCycle = 0;
        }
        else{
            pwmConfigLeft.dutyCycle += 10;
            pwmConfigRight.dutyCycle += 10;
        }
        MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigLeft);
        MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);
    }
    else if(status & GPIO_PIN4){
        if(start == 0){
            start = 1;
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
        }
        else{
            start = 0;
            carStop();
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        }
    }
}
