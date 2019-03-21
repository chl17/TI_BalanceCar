/*
 * Wi-Fi_UART.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */
#include <./Headers/Drivers/Wi-Fi_UART.h>

//Variables in UART IRQHandler to handle data of Wi-fi UART module
uint8_t newLineReceived = 0;
uint8_t receivedString[100] = {0};
int receivedStartByte = 0;
int receivedByteCount = 0;
int receivedStringLength = 0;

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

void WiFi_init(){
    /* Selecting P3.2 and P3.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    //![Simple UART Example]
    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A2_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);
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

//![Simple UART Config]


