/*
* Rochester Institute of Technology
* Department of Computer Engineering
* CMPE 460  Interfacing Digital Electronics
* Thomas Frumusa 
*
* Filename: CarCode.c
*/
#include "msp.h"
#include "Timer32.h"
#include "Common.h"
#include "CortexM.h"
#include "SysTickTimer.h"
#include "ControlPins.h"
#include "led.h"
#include "ADC14.h"
#include "uart.h"
#include "TimerA.h"
#include "Motors.h"
#include "ServoMotor.h"
#include "switches.h"
#include "SysTickTimer.h"
#include "Timer32.h"
#include "TimerA.h"
#include "uart.h"


int main(){
	//init camer
	INIT_Camera();
	//init motors
	
	//init servos
	//init leds for debugging etc
	//init switches for starting
	//
	
	while(TRUE){
			//Read trace
			//Normalize trace to make it smooth
			//Find left and right edge
			//Solve for DR and DL
			//If
			//D R > ( D L + margin), steer right
			//else if
			//D L > ( D R + margin), steer left
			//else go straight
		}
	
	
	
}
