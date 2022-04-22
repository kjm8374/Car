#ifndef __DCMOTOR_H_   /* Include guard */
#define __DCMOTOR_H_

#include "Common.h"
#include <stdint.h>

extern double MaxSpeed;
void reset_PWM(void);
void delay(int del);
void init_motors(uint16_t period);
void init_servos(uint16_t period);
void MotorsForward(double speed);
void LeftMotorForward(double speed);
void RightMotorForward(double speed);
void RightMotorReverse(double speed);
void LeftMotorReverse(double speed);
void MotorsReverse(double speed);

#endif // __DCMOTOR_H_

