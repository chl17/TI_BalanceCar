/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#define pitchyBias 1

//Start or stop flag
int start = 0;

//Systick tick count, increases 1 per 1/1000 s.
long long tick = 0;

//Encoder
//Record the number of rising edge captured
int countLeft = 0, lastCountLeft = 0, countRight = 0, lastCountRight = 0, pauseSpeed = 0;
//Forward or backward flag
uint8_t advanceOrBackLeft = 0, advanceOrBackRight = 0;
//Rotational Speed-N
float speedNLeft = 0, speedNRight = 0;

//Variables in UART IRQHandler to handle data of Wi-fi UART module
uint8_t newLineReceived = 0;
uint8_t receivedString[100] = {0};
uint8_t protocolString[100] = {0};
int receivedStartByte = 0;
int receivedByteCount = 0;
int receivedStringLength = 0;

//Variables in UART IRQHandler for calculation of data from JY901
//W-omega
uint8_t receivedStringG52[11] = {0};
int receivedStartByteG52 = 0;
int receivedByteCountG52 = 0;
//Pitch angle
uint8_t receivedStringG53[11] = {0};
int receivedStartByteG53 = 0;
int receivedByteCountG53 = 0;
float Ax, Ay, Az, Rollx, Pitchy, Yawz, Wx, Wy, Wz;

//Unused declaration
//Record time cost in a certain period of program, e.g. put time1 and time2 at the start and end of a function respectively
int time1 = 0, time2 = 0, time3 = 0, time4 = 0, time5 = 0, time_diff;
float Balance_Kp=223, Balance_Kd=0.82, Velocity_Kp=83, Velocity_Ki=0.41;    //Parameters copied from Minibalance
float Velocity = 50;                                                        //Parameters copied from Minibalance
float Velocity_KP=12, Velocity_KI=12;                                       //Parameters copied from Minibalance
int errorCount = 0, errorCatch = 0;
float speed = 0;
float turnAngle = 0;

//Support for printf()
int  fputc(int _c, register FILE *_fp);
int  fputs(const char *_ptr, register FILE *_fp);


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

//![Simple UART Config]
/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 *http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */

const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // MCLK Clock Source
        1,                                     // BRDIV = 78
        10,                                       // UCxBRF = 2
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

//![Simple UART Config]

//Delay function, delay_time in ms
void delay_ms(int delay_time){
    long long temp = tick;
    while(tick - temp < delay_time);
}

//Copy received data from Wi-Fi to protocolString to avoid being overwritten
void ProtocolCpyData(void){
    memcpy(protocolString, receivedString, receivedStringLength + 1);
    memset(receivedString, 0x00, sizeof(receivedString));
}

//Protocol handler
void Protocol(void){
    ProtocolCpyData();
    newLineReceived = 0;
    memset(protocolString, 0x00, sizeof(protocolString));
}

//Constrain and set PWM
void setPWM(int left,int right){
    int Amplitude = 3000, deadValueLeft = 20, deadValueRight = 60;
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
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigLeft);
    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);
}

void carStop(){
    pwmConfigLeft.dutyCycle = 0;
    pwmConfigRight.dutyCycle = 0;
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigLeft);
    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);
}

void Systick_init(){
    /* Configuring SysTick to trigger at 3 (MCLK is 3MHz so this will
             * make it toggle 1M times every 1s) */

    MAP_CS_initClockSignal(CS_MCLK, CS_MODOSC_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod(24000);
    MAP_SysTick_enableInterrupt();
}

void Motor_init(){
    //Output to TB6612 to control forward or backward
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN4);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN5);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN7);

    /* Configuring GPIO2.4 GPIO5.6 as peripheral output for PWM  */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
            GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN6,
            GPIO_PRIMARY_MODULE_FUNCTION);
}

void Encoder_init(){
    /* Configuring GPIO2.4 GPIO5.6 as peripheral output for PWM  and P2.5 P5.7 for timer
             * capture */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN5,
            GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN7,
            GPIO_PRIMARY_MODULE_FUNCTION);

    //Phase B to determine forward or backward
    MAP_GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN4);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN5);

    /* Configuring Capture Mode */
    MAP_Timer_A_initCapture(TIMER_A0_BASE, &captureModeConfig);
    MAP_Timer_A_initCapture(TIMER_A2_BASE, &captureModeConfig);

    /* Enabling interrupts */
    MAP_Interrupt_enableInterrupt(INT_TA0_N);
    MAP_Interrupt_enableInterrupt(INT_TA2_N);
}

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

void UART_init(){
    /* Selecting P3.2 and P3.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,
            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    //![Simple UART Example]
    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A1_BASE, &uartConfig);
    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A1_BASE);
    MAP_UART_enableModule(EUSCI_A2_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);
}

void Timer32_init(){
    /* Configuring Timer32 to 24000000 (1s) of MCLK in periodic mode */
    MAP_Timer32_initModule(TIMER32_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT,
            TIMER32_PERIODIC_MODE);
    MAP_Timer32_setCount(TIMER32_BASE,120000);

    /* Enabling interrupts */
    MAP_Timer32_enableInterrupt(TIMER32_BASE);
    MAP_Interrupt_enableInterrupt(INT_T32_INT1);

    MAP_Timer32_startTimer(TIMER32_BASE, false);
}

void LED_init(){
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
}

void Interrupt_init(){
    Interrupt_setPriority(INT_T32_INT1, 0);
    MAP_Interrupt_enableMaster();
}

void turnTest(float angle){
    turnAngle = angle;
    delay_ms(1000);
    turnAngle = 0;
    delay_ms(1000);
    turnAngle = -angle;
    delay_ms(1000);
}

void runTest(float Speed){
    speed = Speed;
    delay_ms(1000);
    speed = -Speed;
    delay_ms(1000);
}


int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    Systick_init();
    LED_init();
    Motor_init();
    Encoder_init();
    UART_init();
    Button_init();
    LED_init();
    Timer32_init();
    Interrupt_init();

    /* Sleeping when not in use */
    while(1)
    {
        //turnTest(20);
        //runTest(20);
        //printf("Pitchy: %f", Pitchy);
        //printf("%d %d %d %d %d %d\n", time1, time2, time3, time4, time5, time_diff);
        //printf("speedN: %.2f %.2f\n", speedNLeft, speedNRight);
        //printf("SpeedNLeft: %f, SpeedNRight: %f,\n Ax: %f, Ay: %f, Az: %f,\n Wx: %f, Wy: %f, Wz: %f,\n Rollx: %f, Pitchy: %f, Yawz: %f", speedNLeft, speedNRight, Ax, Ay, Az, Wx, Wy, Wz, Rollx, Pitchy, Yawz);
        //delay_ms(500);
    }
}

/* EUSCI A1 UART ISR  */
void EUSCIA1_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A1_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A1_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG){
        uint8_t receivedByte;
        receivedByte = MAP_UART_receiveData(EUSCI_A1_BASE);

        if(receivedByte == 0x52){
            receivedStartByteG52 = 1;
            receivedByteCountG52 = 0;
        }
        if(receivedStartByteG52 == 1){
            receivedStringG52[receivedByteCountG52++] = receivedByte;
        }
        if(receivedStartByteG52 == 1 && receivedByte == 0x55){
            uint8_t sum = 0x55, i;
            receivedStartByteG52 = 0;
            for(i = 0; i < 9; i++){
                sum += receivedStringG52[i];
            }
            if(sum == receivedStringG52[9]){
                Wx = (short)(((short)receivedStringG52[2] << 8) | receivedStringG52[1]) / 32768.0 * 2000;
                Wy = (short)(((short)receivedStringG52[4] << 8) | receivedStringG52[3]) / 32768.0 * 2000;
                Wz = (short)(((short)receivedStringG52[6] << 8) | receivedStringG52[5]) / 32768.0 * 2000;
            }
        }

        if(receivedByte == 0x53){

            receivedStartByteG53 = 1;
            receivedByteCountG53 = 0;
        }
        if(receivedStartByteG53 == 1){
            receivedStringG53[receivedByteCountG53++] = receivedByte;
        }
        if(receivedStartByteG53 == 1 && receivedByte == 0x55){
            uint8_t sum = 0x55, i;
            receivedStartByteG53 = 0;
            for(i = 0; i < 9; i++){
                sum += receivedStringG53[i];
            }
            if(sum == receivedStringG53[9]){
                Rollx = (short)(((short)receivedStringG53[2] << 8) | receivedStringG53[1]) / 32768.0 * 180.0;
                Pitchy = (short)(((short)receivedStringG53[4] << 8) | receivedStringG53[3]) / 32768.0 * 180.0;
                Yawz = (short)(((short)receivedStringG53[6] << 8) | receivedStringG53[5]) / 32768.0 * 180.0;
            }

        }
    }
}

/* EUSCI A2 UART ISR  */
void EUSCIA2_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG){
        //MAP_UART_transmitData(EUSCI_A2_BASE, MAP_UART_receiveData(EUSCI_A2_BASE));
        uint8_t receivedByte;
        receivedByte = MAP_UART_receiveData(EUSCI_A2_BASE);
        if(receivedByte == '$'){
            receivedStartByte = 1;
            receivedByteCount = 0;
        }
        if(receivedStartByte == 1){
            receivedString[receivedByteCount] = receivedByte;
        }
        if(receivedStartByte == 1 && receivedByte == '\n'){
            newLineReceived = 1;
            receivedStartByte = 0;
            receivedStringLength = receivedByteCount;
        }
        receivedByteCount++;
        if(receivedByteCount >= 100){
            receivedByteCount = 0;
            receivedStartByte = 0;
            newLineReceived = 0;
        }
    }

}

int balance(){
    float Bias, kp = 180, kd = -12;
    int balance;
    if(Pitchy > 50 || Pitchy < -50){
        errorCatch = 1;
        errorCount++;
    }
    else if(errorCatch == 1 && errorCount < 10){
        errorCatch = 0;
        errorCount = 0;
    }

    if(errorCount >= 10){
        start = 0;
        errorCount = 0;
        errorCatch = 0;
    }

    Bias = Pitchy - pitchyBias;
    balance = kp * Bias + Wy * kd ;

    return balance;
}

int velocity(){
    static float Velocity, Encoder_Least, Encoder, Encoder_Integral;
    float kp = 600, ki = kp / 200;
    Encoder_Least = speedNLeft + speedNRight + speed / 40.0;
    Encoder *= 0.70;
    Encoder+= Encoder_Least * 0.30;
    Encoder_Integral += Encoder;
    if(Encoder_Integral > 10000)
        Encoder_Integral = 10000;
    if(Encoder_Integral < -10000)
        Encoder_Integral = -10000;
    Velocity = Encoder * kp + Encoder_Integral * ki;
    return Velocity;
}


int turn(){
    float Turn, kp = 50, Bias;
    Bias = Yawz - turnAngle;
    Turn = - Bias * kp;
    return Turn;
}


/* Timer32 ISR */
void T32_INT1_IRQHandler(void)
{
    float tempSpeedN = 0;
    int PWMLeft, PWMRight;

    tempSpeedN = (countLeft - lastCountLeft) / 15000.0 * 200.0;
    if(tempSpeedN >= 0){
        if(advanceOrBackLeft == 1)
            speedNLeft = tempSpeedN;
        else
            speedNLeft = -tempSpeedN;
    }
    lastCountLeft = countLeft;

    tempSpeedN = (countRight - lastCountRight) / 15000.0 * 200.0;
    if(tempSpeedN >= 0){
        if(advanceOrBackRight == 1)
            speedNRight = -tempSpeedN;
        else
            speedNRight = tempSpeedN;
    }
    lastCountRight = countRight;

    if(start){
        PWMLeft = 1 * balance() - 1 * velocity() + 1 * turn();
        PWMRight = 1 * balance() - 1 * velocity() - 1 * turn();
        setPWM(PWMLeft, PWMRight);
    }
    else
        carStop();

    MAP_Timer32_clearInterruptFlag(TIMER32_BASE);
    //MAP_Timer32_setCount(TIMER32_BASE,120000);
    //MAP_Timer32_startTimer(TIMER32_BASE, true);
}



//static volatile uint_fast16_t timerAcaptureValues;

//******************************************************************************
//
//This is the TIMERA interrupt vector service routine.
//
//******************************************************************************
void TA0_N_IRQHandler(void)
{
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,
            TIMER_A_CAPTURECOMPARE_REGISTER_2);

    advanceOrBackLeft = GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN4);
    countLeft++;

}

void TA2_N_IRQHandler(void)
{
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,
            TIMER_A_CAPTURECOMPARE_REGISTER_2);

    advanceOrBackRight = GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN5);
    countRight++;

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

void SysTick_Handler(void)
{
    tick++;
}

int fputc(int _c, register FILE *_fp){
    while(!(UCA2IFG&UCTXIFG));
    UCA2TXBUF = (unsigned char) _c;
    return((unsigned char)_c);
}

int fputs(const char *_ptr, register FILE *_fp){
    unsigned int i, len;

    len = strlen(_ptr);

    for(i=0 ; i<len ; i++){
        while(!(UCA2IFG&UCTXIFG));
        UCA2TXBUF = (unsigned char) _ptr[i];
    }

    return len;
}
