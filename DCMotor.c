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
	
	TIMER_A0_PWM_Init(period,0.0,1);
	TIMER_A0_PWM_Init(period,0.0,2);
	TIMER_A0_PWM_Init(period,0.0,3);
	TIMER_A0_PWM_Init(period,0.0,4);
}

//	period_servo = 3000000/50; //3 MHz because used clock divider to cut the clock from 48Mhz to 3Mhz
void init_servos(uint16_t period){
	TIMER_A2_PWM_Init(period,0.075,1);
}

//int main(void) {
//	// Initialize UART and PWM
//	// INSERT CODE HERE
//	uint16_t period,period_servo,lab_part;
//	int forward = 0;
//	int phase = 0;
//	double percentDutyCycle = 0.2;
//	lab_part = 1;
//	period_servo = 3000000/50; //3 MHz because used clock divider to cut the clock from 48Mhz to 3Mhz
//	TIMER_A2_PWM_Init(period_servo,0.075,1);
//	period = 48000000/10000;
//	TIMER_A0_PWM_Init(period,percentDutyCycle,1);
//	TIMER_A0_PWM_Init(period,percentDutyCycle,2);
//	TIMER_A0_PWM_Init(period,percentDutyCycle,3);
//	TIMER_A0_PWM_Init(period,percentDutyCycle,4);
//	uart0_init();
//	// Print welcome over serial
//	uart0_put("Running... \n\r");
//	
//	// Generate 20% duty cycle at 10kHz
//	// INSERT CODE HERE
//	if (lab_part==1){
//		TIMER_A0_PWM_DutyCycle(percentDutyCycle,1);
//		TIMER_A0_PWM_DutyCycle(percentDutyCycle,4);
//		for(;;) ;  //then loop forever
//	}
//	else if(lab_part == 2) {
//		while(1)  //loop forever
//		{
//			int i=0;
//			reset_PWM();
//			// 0 to 100% duty cycle in forward direction
//			for (i=0; i<100; i++) {
//					// INSERT CODE HERE
//				TIMER_A0_PWM_DutyCycle((double)i/100,4);
//				delay(10);
//			}
//			uart0_put("0-100 Forward\r\n");			
//			// 100% down to 0% duty cycle in the forward direction
//			reset_PWM();
//			for (i=100; i>=0; i--) {
//					// INSERT CODE HERE
//				TIMER_A0_PWM_DutyCycle((double)i/100,4);
//				delay(10);
//			}
//			uart0_put("100-0 Forward\r\n");		
//			reset_PWM();
//			// 0 to 100% duty cycle in reverse direction
//			for (i=0; i<100; i++) {
//					// INSERT CODE HERE
//				percentDutyCycle = i/100;			
//				TIMER_A0_PWM_DutyCycle((double)i/100,1);
//				delay(10);
//			}
//			uart0_put("0-100 Reverse\r\n");		
//			reset_PWM();
//			// 100% down to 0% duty cycle in the reverse direction
//			for (i=100; i>=0; i--) {
//					// INSERT CODE HERE
//				TIMER_A0_PWM_DutyCycle((double)i/100,1);			
//				delay(10);
//			}
//				uart0_put("100-0 Reverse\r\n");		
//		}
//	}
//	else if (lab_part==3){

//		P4->SEL0 &= ~BIT1; 
//		P4->SEL1 &= ~BIT1;
//		P4->SEL0 &= ~BIT2; 
//		P4->SEL1 &= ~BIT2;
//		P4->SEL0 &= ~BIT3; 
//		P4->SEL1 &= ~BIT3;
//		P4->SEL0 &= ~BIT4; 
//		P4->SEL1 &= ~BIT4;

//		// make built-in LED1 LED high drive strength
//		P4->DS |= BIT1;
//		P4->DS |= BIT2;
//		P4->DS |= BIT3;
//		P4->DS |= BIT4;	
//		// make built-in LED1 out	 
//		P4->DIR |= BIT1;
//		P4->DIR |= BIT2;
//		P4->DIR |= BIT3;
//		P4->DIR |= BIT4;	
//		
//		// turn off LED
//		P4->OUT &= ~BIT1;
//		P4->OUT &= ~BIT2;
//		P4->OUT &= ~BIT3;
//		P4->OUT &= ~BIT4;
//		
//		while(1){
//				P4->OUT &= ~BIT1;
//				P4->OUT &= ~BIT2;
//				P4->OUT &= ~BIT3;
//				P4->OUT &= ~BIT4;
//			//Set one pin high at a tim e 
//			if(forward ){
//				if(phase == 0){P4->OUT |= BIT1; phase ++;} //A , 1a
//				else if(phase == 1){ P4->OUT |= BIT2; phase ++;} //B ,2 a 
//				else if (phase == 2) {P4->OUT |= BIT3; phase ++;} //C ,1b 
//				else {P4->OUT |= BIT4; phase =0;} //D ,2 b
//			}
//			else {//reverse
//				if (phase == 0) {P4->OUT |= BIT4; phase ++;} //D ,2 b
//				else if (phase == 1) {P4->OUT |= BIT3; phase ++;} //C ,1b 
//				else if (phase == 2) {P4->OUT |= BIT2; phase ++;} //B ,2a 
//				else {P4->OUT |= BIT1; phase =0;} //A ,1 a
//			}
//		delay (10); //sm aller values = faster speed
//		}
//	
//	}

//	else if (lab_part==4){
//			while(1) {
//				TIMER_A2_PWM_DutyCycle(0.075,1);
//				delay(100);
//				TIMER_A2_PWM_DutyCycle(0.05,1);
//				delay(100);
//				TIMER_A2_PWM_DutyCycle(0.1,1);
//			}
//	}
//	else {
//		while(1)  //loop forever
//		{
//			int i=0;
//			reset_PWM();
//					// INSERT CODE HERE
//			uart0_put("Forward\r\n");
//			TIMER_A0_PWM_DutyCycle(0.3,1);
//			TIMER_A0_PWM_DutyCycle(0.3,3);	
//			TIMER_A2_PWM_DutyCycle(0.05,1);	
//			delay(100);
//			TIMER_A2_PWM_DutyCycle(0.075,1);
//			delay(100);
//			TIMER_A2_PWM_DutyCycle(0.1,1);
//			reset_PWM();	
//			uart0_put("Reverse\r\n");
//			TIMER_A0_PWM_DutyCycle((double)i/100,2);
//			TIMER_A0_PWM_DutyCycle((double)i/100,4);		
//			TIMER_A2_PWM_DutyCycle(0.05,1);	
//			delay(100);
//			TIMER_A2_PWM_DutyCycle(0.075,1);
//			delay(100);
//			TIMER_A2_PWM_DutyCycle(0.1,1);
//		}
//	}
//	// return 0;
//}



