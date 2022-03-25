#ifndef __TIMERA_H_   /* Include guard */
#define __TIMERA_H_

#include "Common.h"
#include <stdint.h>

int TIMER_A0_PWM_Init(uint16_t period, double percentDutyCycle, uint16_t pin);
void TIMER_A0_PWM_DutyCycle(double percentDutyCycle, uint16_t pin);
int TIMER_A2_PWM_Init(uint16_t period, double percentDutyCycle, uint16_t pin);
void TIMER_A2_PWM_DutyCycle(double percentDutyCycle, uint16_t pin);

#endif // __TIMERA_H_
