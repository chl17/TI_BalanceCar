/*
 * Interrupt.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */
#include <./Headers/Drivers/Interrupt.h>

void Interrupt_init(){
    //Set Timer32 interrupt to the highest priority
    Interrupt_setPriority(INT_T32_INT1, 0);
    //Enable all interrupt
    Interrupt_enableMaster();
}
