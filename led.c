#include "msp.h" 
#include<stdio.h>
#include<stdlib.h>
#include <time.h>


typedef unsigned char BOOLEAN;

#define FALSE 0
#define TRUE  !FALSE

#define RED         BIT0  
#define GREEN       BIT1
#define BLUE        BIT2

void LED1_Init(void)
{
	// configure PortPin for LED1 as port I/O 
	P1->SEL0 &= ~RED; 
	P1->SEL1 &= ~RED;

	// make built-in LED1 LED high drive strength
	P1->DS |= RED;

	// make built-in LED1 out	 
	P1->DIR |= RED;
	
	// turn off LED
	P1->OUT &= ~RED;
}

void LED2_Init(void)
{
	// configure PortPin for LED2 as port I/O 
	//LED2-Red is port2.0
	P2->SEL0 &= ~RED;
	P2->SEL1 &= ~RED;
	
	//LED2-Green is port 2.1
	P2->SEL0 &= ~GREEN;
	P2->SEL1 &= ~GREEN;
	
	//LED2-Blue is port 2.2
	P2->SEL0 &= ~BLUE;
	P2->SEL1 &= ~BLUE;

	// make built-in LED2 LEDs high drive strength
	P2->DS |= RED;
	P2->DS |= GREEN;
	P2->DS |= BLUE;

	// make built-in LED2 out	 
	P2->DIR |= RED;
	P2->DIR |= GREEN;
	P2->DIR |= BLUE;

	// turn off LED
	P2->OUT &= ~RED;
	P2->OUT &= ~GREEN;
	P2->OUT &= ~BLUE;
 
}

int LED1_State(void){
	if (P1->OUT & BIT0){
		return TRUE;
	}else{
		return FALSE;
	}
}

void LED1_On(void){
	P1->OUT |= RED;
}

void LED1_Off(void){
	P1->OUT &= ~RED;
}

void LED2_Off(void){
	P2->OUT &= ~RED;
	P2->OUT &= ~GREEN;
	P2->OUT &= ~BLUE;
}
void LED2_RED_ON(void){
	P2->OUT |= RED;
	P2->OUT &= ~GREEN;
	P2->OUT &= ~BLUE;
}
void LED2_GREEN_ON(void){
	P2->OUT &= ~RED;
	P2->OUT |= GREEN;
	P2->OUT &= ~BLUE;
}
void LED2_BLUE_ON(void){
	P2->OUT &= ~RED;
	P2->OUT &= ~GREEN;
	P2->OUT |= BLUE;
}
void LED2_CYAN_ON(void){
	P2->OUT &= ~RED;
	P2->OUT |= GREEN;
	P2->OUT |= BLUE;
}

void LED2_MAGENTA_ON(void){
	P2->OUT |= RED;
	P2->OUT &= ~GREEN;
	P2->OUT |= BLUE;
}

void LED2_YELLOW_ON(void){
	P2->OUT |= RED;
	P2->OUT |= GREEN;
	P2->OUT &= ~BLUE;
}

void LED2_WHITE_ON(void){
	P2->OUT |= RED;
	P2->OUT |= GREEN;
	P2->OUT |= BLUE;
}

