/*
 * Protocol.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */
#include <./Headers/Users/Protocol.h>

uint8_t protocolString[100] = {0};

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
