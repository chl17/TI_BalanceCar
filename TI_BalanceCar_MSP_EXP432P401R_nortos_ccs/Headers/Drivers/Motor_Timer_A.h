/*
 * Motor_Timer_A.h
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */

#ifndef HEADERS_DRIVERS_MOTOR_TIMER_A_H_
#define HEADERS_DRIVERS_MOTOR_TIMER_A_H_

#include <./Headers/library.h>
#include <./Headers/parameters.h>

//![Simple Timer_A Config]
/* Timer_A PWM Configuration Parameter */
extern Timer_A_PWMConfig pwmConfigLeft;
extern Timer_A_PWMConfig pwmConfigRight;

void Motor_init();

//Constrain and set PWM
void setPWM(int left,int right);

#endif /* HEADERS_DRIVERS_MOTOR_TIMER_A_H_ */
