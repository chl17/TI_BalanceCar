/*
 * Functions.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */
#include <./Headers/Users/Functions.h>

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
