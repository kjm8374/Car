/*
 * Main module for testing the PWM Code for the K64F
 * 
 * Author:  
 * Created:  
 * Modified: Carson Clarke-Magrab <ctc7359@rit.edu> 
 * LJBeato
 * 2021
 */

#include "msp.h"
#include "uart.h"
#include "TimerA.h"


/**
 * Waits for a delay (in milliseconds)
 * 
 * del - The delay in milliseconds
 */
//void delay(int del){
//	volatile int i;
//	for (i=0; i<del*50000; i++){
//		;// Do nothing
//	}
//}

void reset_PWM(){
	
		TIMER_A0_PWM_DutyCycle(0.0,1);
		TIMER_A0_PWM_DutyCycle(0.0,2);
		TIMER_A0_PWM_DutyCycle(0.0,3);
		TIMER_A0_PWM_DutyCycle(0.0,4);	
}

void init_motors(uint16_t period){
	
	// configure PortPin for jumper board to enable
	//both motors need to turn on 3.6 and 3.7
	//set as general i/o
	P3->SEL0 &= ~BIT6; 
	P3->SEL1 &= ~BIT6;
	P3->SEL0 &= ~BIT7; 
	P3->SEL1 &= ~BIT7;
	// make 3.6 and 3.7 high drive strength
	P3->DS |= BIT6;
	P3->DS |= BIT7;
	// make 3.6 and 3.7 outputs	 
	P3->DIR |= BIT6;
	P3->DIR |= BIT7;
	// turn on output 3.6 and 3.7
	P3->OUT |= BIT6;
	P3->OUT |= BIT7;
	
	//the rest of motor initialization needed
	TIMER_A0_PWM_Init(period,0.0,1);
	TIMER_A0_PWM_Init(period,0.0,2);
	TIMER_A0_PWM_Init(period,0.0,3);
	TIMER_A0_PWM_Init(period,0.0,4);
}

//	period_servo = 3000000/50; //3 MHz because used clock divider to cut the clock from 48Mhz to 3Mhz
void init_servos(uint16_t period){
	TIMER_A2_PWM_Init(period,0.075,1);
}

void MotorsForward(double speed){
			reset_PWM();
			speed = speed*0.01;
		  TIMER_A0_PWM_DutyCycle(speed,1);
  		TIMER_A0_PWM_DutyCycle(speed,3);
  		TIMER_A0_PWM_DutyCycle(0.0,2);
  		TIMER_A0_PWM_DutyCycle(0.0,4);
}

void MotorsReverse(double speed){
	    reset_PWM();
			speed = speed*0.01;
		  TIMER_A0_PWM_DutyCycle(speed,2);
  		TIMER_A0_PWM_DutyCycle(speed,4);
  		TIMER_A0_PWM_DutyCycle(0.0,1);
  		TIMER_A0_PWM_DutyCycle(0.0,3);
}



