#ifndef __DCMOTOR_H_   /* Include guard */
#define __DCMOTOR_H_

#include "Common.h"
#include <stdint.h>

void reset_PWM(void);
void delay(int del);
void init_motors(uint16_t period);
void init_servos(uint16_t period);
void MotorsForward(double speed);
void LeftMotorForward(double speed);
void RightMotorForward(double speed);
void MotorsReverse(double speed);

#endif // __DCMOTOR_H_

