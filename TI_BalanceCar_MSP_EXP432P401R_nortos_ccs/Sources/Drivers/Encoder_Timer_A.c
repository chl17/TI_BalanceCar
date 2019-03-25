/*
 * Encoder_Timer_A.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */

#include <./Headers/Drivers/Encoder_Timer_A.h>

//Encoder
//Record the number of rising edge captured
int countLeft = 0, lastCountLeft = 0, countRight = 0, lastCountRight = 0;
//Forward or backward flag
uint8_t advanceOrBackLeft = 0, advanceOrBackRight = 0;
//Rotational Speed-N
float speedNLeft = 0, speedNRight = 0;

/* Timer_A Capture Mode Configuration Parameter */
const Timer_A_CaptureModeConfig captureModeConfig =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_2,        // CC Register 2
        TIMER_A_CAPTUREMODE_RISING_EDGE,          // Rising Edge
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // CCIxB Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
};

void Encoder_init(){
    /* Configuring GPIO2.4 GPIO5.6 as peripheral output for PWM  and P2.5 P5.7 for timer
             * capture */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN5,
            GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN7,
            GPIO_PRIMARY_MODULE_FUNCTION);

    //Phase B to determine forward or backward
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN4);
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN5);

    /* Configuring Capture Mode */
    Timer_A_initCapture(TIMER_A0_BASE, &captureModeConfig);
    Timer_A_initCapture(TIMER_A2_BASE, &captureModeConfig);

    /* Enabling interrupts */
    Interrupt_enableInterrupt(INT_TA0_N);
    Interrupt_enableInterrupt(INT_TA2_N);
}

//******************************************************************************
//
//This is the TIMERA interrupt vector service routine.
//
//******************************************************************************
void TA0_N_IRQHandler(void)
{
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,
            TIMER_A_CAPTURECOMPARE_REGISTER_2);
    //Left motor Phase B
    advanceOrBackLeft = GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN4);
    countLeft++;

}

void TA2_N_IRQHandler(void)
{
    Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,
            TIMER_A_CAPTURECOMPARE_REGISTER_2);
    //Right motor Phase B
    advanceOrBackRight = GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN5);
    countRight++;

}

