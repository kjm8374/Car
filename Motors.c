#include "msp.h"
#include "uart.h"
#include "TimerA.h"

void delay(int del){
	volatile int i;
	for (i=0; i<del*50000; i++){
		;// Do nothing
	}
}

int main(){
	int forward = 0;
	int phase = 0;
	P4->SEL0 &= ~BIT1; 
	P4->SEL1 &= ~BIT1;
	P4->SEL0 &= ~BIT2; 
	P4->SEL1 &= ~BIT2;
	P4->SEL0 &= ~BIT3; 
	P4->SEL1 &= ~BIT3;
	P4->SEL0 &= ~BIT4; 
	P4->SEL1 &= ~BIT4;

	// make built-in LED1 LED high drive strength
	P4->DS |= BIT1;
	P4->DS |= BIT2;
	P4->DS |= BIT3;
	P4->DS |= BIT4;	
	// make built-in LED1 out	 
	P4->DIR |= BIT1;
	P4->DIR |= BIT2;
	P4->DIR |= BIT3;
	P4->DIR |= BIT4;	
	
	// turn off LED
	P4->OUT &= ~BIT1;
	P4->OUT &= ~BIT2;
	P4->OUT &= ~BIT3;
	P4->OUT &= ~BIT4;
	
	while(1){
			P4->OUT &= ~BIT1;
			P4->OUT &= ~BIT2;
			P4->OUT &= ~BIT3;
			P4->OUT &= ~BIT4;
		//Set one pin high at a tim e 
		if(forward ){
			if(phase == 0){P4->OUT |= BIT1; phase ++;} //A , 1a
			else if(phase == 1){ P4->OUT |= BIT2; phase ++;} //B ,2 a 
			else if (phase == 2) {P4->OUT |= BIT3; phase ++;} //C ,1b 
			else {P4->OUT |= BIT4; phase =0;} //D ,2 b
		}
		else {//reverse
			if (phase == 0) {P4->OUT |= BIT4; phase ++;} //D ,2 b
			else if (phase == 1) {P4->OUT |= BIT3; phase ++;} //C ,1b 
			else if (phase == 2) {P4->OUT |= BIT2; phase ++;} //B ,2a 
			else {P4->OUT |= BIT1; phase =0;} //A ,1 a
		}
//Note - you need to write your own delay function 
	delay (10); //sm aller values = faster speed
	}
}
