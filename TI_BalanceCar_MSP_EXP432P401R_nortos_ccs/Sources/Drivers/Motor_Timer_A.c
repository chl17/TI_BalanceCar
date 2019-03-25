/*
 * PWM_Timer_A.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */
#include <./Headers/Drivers/Motor_Timer_A.h>

//![Simple Timer_A Config]
/* Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig pwmConfigLeft =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        3000,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0
};
Timer_A_PWMConfig pwmConfigRight =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        3000,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0
};

void Motor_init(){
    //Output to TB6612 to control forward or backward
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN5);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN7);

    /* Configuring GPIO2.4 GPIO5.6 as peripheral output for PWM  */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
            GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN6,
            GPIO_PRIMARY_MODULE_FUNCTION);
}

//Constrain input value and set PWM
void setPWM(int left,int right){
    int Amplitude = 3000, deadValueLeft = deadValueL, deadValueRight = deadValueR;
    if(left >= 0){
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4);
        if(left + deadValueLeft> 3000)
            pwmConfigLeft.dutyCycle = Amplitude;
        else
            pwmConfigLeft.dutyCycle = left + deadValueLeft;
    }
    else{
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);
        if(left - deadValueLeft < -3000)
            pwmConfigLeft.dutyCycle = Amplitude;
        else
            pwmConfigLeft.dutyCycle = -left + deadValueLeft;
    }

    if(right >= 0){
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);
        if(right + deadValueRight > 3000)
            pwmConfigRight.dutyCycle = Amplitude;
        else
            pwmConfigRight.dutyCycle = right + deadValueRight;
    }
    else{
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN7);
        if(right - deadValueRight < -3000)
            pwmConfigRight.dutyCycle = Amplitude;
        else
            pwmConfigRight.dutyCycle = -right + deadValueRight;
    }
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigLeft);
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);
}
