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
#include "DCMotor.h"
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
#include "Camera.h"
#include "uart.h"
#include <stdio.h>

extern uint16_t line[128];

int main(){
	//init camera
	char str[200];
	uint16_t motor_period = 48000000/10000;
	uint16_t servo_period = 3000000/50;
	DisableInterrupts();
	uart0_init();
	//init leds for debugging etc
	LED1_Init();
	LED2_Init();
	INIT_Camera();
	//init motors
	init_motors(motor_period);
	//init servos
	init_servos(servo_period);

	//init switches for starting
	//
	
	while(TRUE){
			//Read trace
		for (int i=0; i < 129; i++){
			sprintf(str, "%d", line[i]);
			uart0_put(str);
		}
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
