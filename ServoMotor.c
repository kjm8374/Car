
#include "msp.h"
#include "uart.h"
#include "TimerA.h"

void delay(int del){
	volatile int i;
	for (i=0; i<del*50000; i++){
		;// Do nothing
	}
}

int main(void) {
	
	uint16_t period;
	period = 3000000/50; //3 MHz because used clock divider to cut the clock from 48Mhz to 3Mhz
	TIMER_A2_PWM_Init(period,0.075,1);
	while(1) {
		TIMER_A2_PWM_DutyCycle(0.075,1);
		delay(100);
		TIMER_A2_PWM_DutyCycle(0.05,1);
		delay(100);
		TIMER_A2_PWM_DutyCycle(0.1,1);
	}
	
	return 0;
}

