/*
 * Control.c
 *
 *  Created on: 2019/3/21
 *      Author: Lewis Chen
 */
#include <./Headers/Users/Control.h>

//Start or stop flag
int start = 0;
/* Wrong data from the gyroscope may be received to make the car stop,
     this records the number of data */
int errorCount = 0;
/*The expected speed and turnAngle,
 * your functions should change the two variables to control the car */
float speed = 0;
float turnAngle = 0;

void carStop(){
    pwmConfigLeft.dutyCycle = 0;
    pwmConfigRight.dutyCycle = 0;
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigLeft);
    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);
}

//Balance PID
int balance(){
    float Bias, kp = Balance_Kp, kd = Balance_Kd;
    int balance;
    if(Pitchy > 50 || Pitchy < -50){
        errorCount++;
    }
    else if(errorCount < 10){
        errorCount = 0;
    }

    if(errorCount >= 10){
        start = 0;
        errorCount = 0;
    }

    Bias = Pitchy - pitchyBias;
    balance = kp * Bias + Wy * kd ;

    return balance;
}

//Velocity PID
int velocity(){
    static float Velocity, Encoder_Least, Encoder, Encoder_Integral;
    float kp = Velocity_Kp, ki = kp / 200;
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

//Turning PID
int turn(){
    float Turn, kp = Turn_Kp, Bias;
    Bias = Yawz - turnAngle;
    Turn = - Bias * kp;
    return Turn;
}


