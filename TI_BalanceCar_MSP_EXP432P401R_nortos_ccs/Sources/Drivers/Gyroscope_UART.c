/*
 * Gyroscope_UART.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */
#include <./Headers/Drivers/Gyroscope_UART.h>

//Variables in UART IRQHandler for calculation of data from JY901
//W-omega
uint8_t receivedStringG52[11] = {0};
int receivedStartByteG52 = 0;
int receivedByteCountG52 = 0;
//Pitch angle
uint8_t receivedStringG53[11] = {0};
int receivedStartByteG53 = 0;
int receivedByteCountG53 = 0;
float Rollx, Pitchy, Yawz, Wx, Wy, Wz;

//![Simple UART Config]
/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 *http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const eUSCI_UART_Config uartConfig_Gyro =
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

void UART_init(){
    /* Selecting P3.2 and P3.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,
            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    //![Simple UART Example]
    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A1_BASE, &uartConfig_Gyro);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A1_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
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
