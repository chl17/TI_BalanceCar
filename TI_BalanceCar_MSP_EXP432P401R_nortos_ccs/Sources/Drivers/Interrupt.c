/*
 * Interrupt.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */
#include <./Headers/Drivers/Interrupt.h>

void Interrupt_init(){
    Interrupt_setPriority(INT_T32_INT1, 0);
    MAP_Interrupt_enableMaster();
}
