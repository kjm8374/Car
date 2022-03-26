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

int currentLocation = 0;
int leftPeakLoc = 0;
int rightPeakLoc = 0;
int dif =0;
float CalculatedPosition =65;
int trackWidth =128;


extern uint16_t line[128];
uint16_t SmoothLine[128];
uint16_t DiffLine[128];

void CalculatePeakLocations(void);
void evaluatePositionANDTurn(int LeftLocation, int RightLocation);
void FilterLine(void);
void myDelay(int del);


//should be delay in ms (del)
void myDelay(int del){
	volatile int i;
	for (i=0; i<del*50000; i++){
		;// Do nothing
	}
}

int main(){
	//init camera
	char str[200];
	int i=0;
	uint16_t motor_period = 48000000/10000;
	uint16_t servo_period = 3000000/50;
	//DisableInterrupts();
	uart0_init();
	uart0_put("Uart Initialized\r\n");
	//init leds for debugging etc
	LED1_Init();
	LED2_Init();
	INIT_Camera();
	//init motors
	init_motors(motor_period);
	//init servos
	init_servos(servo_period);
	//EnableInterrupts();
////testing for servos uncomment to test that bad boy
//	while(i<5) {
//		TIMER_A2_PWM_DutyCycle(0.075,1);
//		myDelay(100);
//		TIMER_A2_PWM_DutyCycle(0.05,1);
//		myDelay(100);
//		TIMER_A2_PWM_DutyCycle(0.1,1);
//		myDelay(100);
//		i++;
//			}
//	i=0;
//	myDelay(1000);
//		reset_PWM();

////testing for dc motors uncomment to test those bad boys
//while(TRUE){
//	//forward
//	//uart0_put("forward\n");

//	TIMER_A0_PWM_DutyCycle(0.4,1);
//	TIMER_A0_PWM_DutyCycle(0.4,3);
//	TIMER_A0_PWM_DutyCycle(0.0,2);
//	TIMER_A0_PWM_DutyCycle(0.0,4);
//	myDelay(1000);

//	reset_PWM();
//	myDelay(100);
//	//reverse
//	
//	//uart0_put("forward\n");
//	TIMER_A0_PWM_DutyCycle(0.4,2);
//	TIMER_A0_PWM_DutyCycle(0.4,4);
//	TIMER_A0_PWM_DutyCycle(0.0,1);
//	TIMER_A0_PWM_DutyCycle(0.0,3);
//	myDelay(1000);
//	reset_PWM();
//	
//}	
	//init switches for starting
	
	
	//
	//main while loop where all the magic happens
	while(TRUE){
			//Read trace
		for (i=0; i < 129; i++){
			sprintf(str, "%d\r\n", line[i]);
			uart0_put(str);
		}	
		//Normalize trace to make it smooth
		//uses line to generate smoothline and diffline
		FilterLine();
		
		//Find left and right edge
		// gets the peak locations dawg 
		CalculatePeakLocations();

		//takes in left and right location
		//figure out where car is on track and
		//how much to turn and which direction to turn etc
		//Solve for DR and DL
		//If
		//D R > ( D L + margin), steer right
		//else if
		//D L > ( D R + margin), steer left
		//else go straight
		evaluatePositionANDTurn(leftPeakLoc,rightPeakLoc);

		}
	
	
	
}

// find the locations of the peaks 
void CalculatePeakLocations(){
	int idx;
	int TempVal = 999;
	leftPeakLoc = 64;
	rightPeakLoc = 64;
	
	//might want to change this to drop first 20 and last 20 instead of 10
	for(idx = 10;idx <129; idx++){
		TempVal = DiffLine[idx];
		//edge case
		if(TempVal > DiffLine[leftPeakLoc])
		{
			leftPeakLoc = idx;
		}
		//edge case
		if(TempVal<DiffLine[rightPeakLoc])
		{
			rightPeakLoc=idx;
		}
	}
		//actual case
		if(leftPeakLoc>=64){
			leftPeakLoc=64;
			rightPeakLoc=127;
		}
		//actual case
		else if(rightPeakLoc<=64){
			leftPeakLoc=0;
			rightPeakLoc = 64;
		}
}


void FilterLine(){
	int L = 0;
	
	for(L=2;L<127;L++){
		SmoothLine[L] = (((line[L-2])+ (line[L-1]) + (line[L]) + (line[L+1]) + (line[L+2]))/6);
		//SmoothLine[L+1] = (((line[L-1])+ (line[L]) + (line[L+1]) + (line[L+2]) + (line[L+3]))/6);	
		DiffLine[L-1] = ((SmoothLine[L+1] - SmoothLine[L-1])/2);		
	}
		//DiffLine[126] = (SmoothLine[127] - SmoothLine[125])/2;
		DiffLine[127] = (SmoothLine[127] - SmoothLine[126]);
	
}

void evaluatePositionANDTurn(int LeftLocation, int RightLocation){
	int halfTrack = trackWidth >> 2;
	int LeftMiss = 0;
	int RightMiss = 0;
	if(LeftLocation == 999 && RightLocation == 999){
		CalculatedPosition = 64;
	}
	else{
		if(RightMiss ==0 && LeftMiss == 0){
			//means that we found both lines
			dif = RightLocation - LeftLocation;
			currentLocation = dif/2;
			CalculatedPosition = currentLocation + LeftLocation;
		}
		else if(RightMiss ==0 && LeftMiss == 1){
			//cant find left but found the right
			//verify its actually an edge
			CalculatedPosition = (RightLocation - halfTrack);
		}
		else if(RightMiss==1 && LeftMiss ==0){
			CalculatedPosition = (LeftLocation + halfTrack);
		}
	
	}	
	
	//on left of the track, need to TURN RIGHT
	if(CalculatedPosition< 64 ){
		//entercode to turn right
		TIMER_A2_PWM_DutyCycle(0.1,1);
		
	}
		//on right of the track, need to TURN LEFT
	if(CalculatedPosition>64 ){
		//enter code to turn left
		TIMER_A2_PWM_DutyCycle(0.05,1);
		
	}
	//in center of track, not very likely
	if(CalculatedPosition==64 ){
		//enter code to turn left
		TIMER_A2_PWM_DutyCycle(0.075,1);
		
	}

}	
			
