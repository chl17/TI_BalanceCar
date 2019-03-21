/*
 * Encoder_Timer_A.h
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */

#ifndef HEADERS_DRIVERS_ENCODER_TIMER_A_H_
#define HEADERS_DRIVERS_ENCODER_TIMER_A_H_

#include <./Headers/library.h>

//Encoder
//Record the number of rising edge captured
extern int countLeft, lastCountLeft, countRight, lastCountRight;
//Forward or backward flag
extern uint8_t advanceOrBackLeft, advanceOrBackRight;

//Rotational Speed-N
extern float speedNLeft, speedNRight;

extern const Timer_A_CaptureModeConfig captureModeConfig;

void Encoder_init();
void TA0_N_IRQHandler(void);
void TA2_N_IRQHandler(void);

#endif /* HEADERS_DRIVERS_ENCODER_TIMER_A_H_ */
