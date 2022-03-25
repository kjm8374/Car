#include "msp.h"
#include "Timer32.h"
#include "Common.h"
#include "CortexM.h"

#define S1          BIT1
#define S2          BIT4

//initializes switch 1 to be a general i/o input
void Switch1_Init(){
	// configure PortPin for Switch2 as port I/O 
	P1->SEL0 &= ~S1;
	P1->SEL1 &= ~S1;
	// configure as input
	//set dir = 0
	P1->DIR &= ~S1;
	//set ren = 1
	P1->REN |= S1;
	//set out=1
	P1->OUT |= S1;  

}

//initializes switch 2 to be a general i/o input
void Switch2_Init(){
	// configure PortPin for Switch2 as port I/O 
	P1->SEL0 &= ~S2;
	P1->SEL1 &= ~S2;
	// configure as input
	//set dir = 0
	P1->DIR &= ~S2;
	//set ren = 1
	P1->REN |= S2;
	//set out=1
	P1->OUT |= S2; 
	
}
