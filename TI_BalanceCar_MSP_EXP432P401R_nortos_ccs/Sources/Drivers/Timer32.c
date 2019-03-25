/*
 * Timer32.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */
#include <./Headers/Drivers/Timer32.h>

void Timer32_init(){
    /* Configuring Timer32 to 24000000 (1s) of MCLK in periodic mode */
    Timer32_initModule(TIMER32_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT,
            TIMER32_PERIODIC_MODE);
    Timer32_setCount(TIMER32_BASE,120000);

    /* Enabling interrupts */
    Timer32_enableInterrupt(TIMER32_BASE);
    Interrupt_enableInterrupt(INT_T32_INT1);

    Timer32_startTimer(TIMER32_BASE, false);
}

/* Timer32 ISR */
void T32_INT1_IRQHandler(void)
{
    float tempSpeedN = 0;
    int PWMLeft, PWMRight;

    //Calculate the rotational speed of left motor
    tempSpeedN = (countLeft - lastCountLeft) / 15000.0 * 200.0;
    if(tempSpeedN >= 0){
        if(advanceOrBackLeft == 1)
            speedNLeft = tempSpeedN;
        else
            speedNLeft = -tempSpeedN;
    }
    lastCountLeft = countLeft;

    //Calculate the rotational speed of left motor
    tempSpeedN = (countRight - lastCountRight) / 15000.0 * 200.0;
    if(tempSpeedN >= 0){
        if(advanceOrBackRight == 1)
            speedNRight = -tempSpeedN;
        else
            speedNRight = tempSpeedN;
    }
    lastCountRight = countRight;

    //PID cascade control
    if(start){
        PWMLeft = 1 * balance() - 1 * velocity() + 1 * turn();
        PWMRight = 1 * balance() - 1 * velocity() - 1 * turn();
        setPWM(PWMLeft, PWMRight);
    }
    else
        carStop();

    Timer32_clearInterruptFlag(TIMER32_BASE);

}
