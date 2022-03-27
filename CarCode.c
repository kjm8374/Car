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
extern BOOLEAN g_sendData;
uint16_t SmoothLine[128];
uint32_t DiffLine[128];
BOOLEAN debug;

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
	//uint16_t servo_period = 3000000/50;
	uint16_t servo_period = 48000000/(50.0*8.0);
	debug = FALSE;
	DisableInterrupts();
	uart0_init();
	uart0_put("Uart Initialized\r\n");
	//init leds for debugging etc
	LED1_Init();
	LED2_Init();
	INIT_Camera();
	uart0_put("1\r\n");
	//init motors
	init_motors(motor_period);
	//init servos
	init_servos(servo_period);
	EnableInterrupts();
	TIMER_A2_PWM_DutyCycle(0.075,1);
	myDelay(25);
	
////testing for servos uncomment to test that bad boy
//	while(TRUE) {
//		TIMER_A2_PWM_DutyCycle(0.045,1);
//		myDelay(25);
//		TIMER_A2_PWM_DutyCycle(0.09,1);
//		myDelay(25);
//		TIMER_A2_PWM_DutyCycle(0.12,1);
//		myDelay(25);
//		i++;
//			}
//	i=0;
//	myDelay(1000);
//		reset_PWM();

////testing for dc motors uncomment to test those bad boys
//	while(TRUE){
//		//forward
//		//uart0_put("forward\n");

//		TIMER_A0_PWM_DutyCycle(0.4,1);
//		TIMER_A0_PWM_DutyCycle(0.4,3);
//		TIMER_A0_PWM_DutyCycle(0.0,2);
//		TIMER_A0_PWM_DutyCycle(0.0,4);
//		myDelay(1000);

//		reset_PWM();
//		myDelay(100);
//		//reverse
//		
//		//uart0_put("forward\n");
//		TIMER_A0_PWM_DutyCycle(0.4,2);
//		TIMER_A0_PWM_DutyCycle(0.4,4);
//		TIMER_A0_PWM_DutyCycle(0.0,1);
//		TIMER_A0_PWM_DutyCycle(0.0,3);
//		myDelay(1000);
//		reset_PWM();
//		
//	}	
	//init switches for starting
	
	//uart0_put("We made it/r/n");
	//
	//main while loop where all the magic happens
	while(TRUE){
		  TIMER_A0_PWM_DutyCycle(0.25,1);
  		TIMER_A0_PWM_DutyCycle(0.25,3);
  		TIMER_A0_PWM_DutyCycle(0.0,2);
  		TIMER_A0_PWM_DutyCycle(0.0,4);
			//Read trace
//		for (i=0; i < 129; i++){
//			sprintf(str, "%d\r\n", line[i]);
//			uart0_put(str);
//		}	
		//Normalize trace to make it smooth
		//uses line to generate smoothline and diffline
		FilterLine();
		if(debug) {
			if (g_sendData == TRUE) 
		{
			LED1_On();
			// send the array over uart
			sprintf(str,"%i\n\r",-1); // start value
			uart0_put(str);
			for (i = 0; i < 128; i++) 
			{
				sprintf(str,"%i\n\r", DiffLine[i]);
				uart0_put(str);
			}
			sprintf(str,"%i\n\r",-2); // end value
			uart0_put(str);
			LED1_Off();
			g_sendData = FALSE;
		}
		}
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
	//tried these starting at 20 but didnt work
	leftPeakLoc = 64;
	rightPeakLoc = 64;
	
	//might want to change this to drop first 20 and last 20 instead of 10
	for(idx = 20;idx <109; idx++){
		TempVal = DiffLine[idx];
		//Actual case
		if(TempVal > DiffLine[leftPeakLoc])
		{
			leftPeakLoc = idx;
		}
		//Actual case
		if(TempVal<DiffLine[rightPeakLoc])
		{
			rightPeakLoc=idx;
		}
	}
//		//Edge case
//		if(leftPeakLoc>=64){
//			//leftPeakLoc=64;
//			rightPeakLoc=127;
//		}
//		//Edge case
//		else if(rightPeakLoc<=64){
//			leftPeakLoc=0;
//			//rightPeakLoc = 64;
//		}
}


void FilterLine(){
	int L = 0;
	
	
	for(L=2;L<127;L++){
		SmoothLine[L] = (((line[L-2])+ (line[L-1]) + (line[L]) + (line[L+1]) + (line[L+2]))/5);
		SmoothLine[L+1] = (((line[L-1])+ (line[L]) + (line[L+1]) + (line[L+2]) + (line[L+3]))/5);	
		DiffLine[L] = ((SmoothLine[L+1] - SmoothLine[L-1])/2);		
	}
		DiffLine[126] = (SmoothLine[127] - SmoothLine[125])/2;
		DiffLine[127] = (SmoothLine[127] - SmoothLine[126]);
	
}

void evaluatePositionANDTurn(int LeftLocation, int RightLocation){
	char streval[200];
	int i =0;
	int halfTrack = trackWidth/2;
	int LeftMiss = 1;
	int RightMiss = 1;
	//int RightFound = 0;//1 for found 0 for not found
	//int LeftFound = 0;//1 for found 0 for not found
	for(i=64; i>0;i--){
		if(DiffLine[i] > 2000){
			//means we found the left line
			//LeftFound=1;
			LeftMiss =0;
		}
	}
	
		for(i=64; i<128;i++){
		if(DiffLine[i] >2000){
			//means we found the right line
			//RightFound=1;
			RightMiss =0;
		}
	}
		
	if(LeftLocation == 999 && RightLocation == 999){
		CalculatedPosition = 64;
	}
	else{
		if(RightMiss ==0 && LeftMiss == 0){
			//means that we found both lines
			dif = RightLocation - LeftLocation;
			currentLocation = dif/2;
			CalculatedPosition = LeftLocation + currentLocation;
		}
		else if(RightMiss ==0 && LeftMiss == 1){
			//cant find left but found the right
			//verify its actually an edge
			CalculatedPosition = (RightLocation - 64);
		}
		else if(RightMiss==1 && LeftMiss ==0){
			CalculatedPosition = (LeftLocation + 64);
		}
	
	}	
	sprintf(streval,"%f\n\r",CalculatedPosition); // start value
	uart0_put(streval);
	//on left of the track, need to TURN RIGHT
	if(CalculatedPosition< 64 )
		{
		//entercode to turn right
		//TIMER_A2_PWM_DutyCycle(0.05,1);
		//myDelay(25);
			//adding better logic here for different turn amounts
			if(CalculatedPosition> 60 ){
				//turn slightly bc close to middle (betweem 60 and 64
				
			}else if(CalculatedPosition> 55 ){
				//turn slightly bc close to middle (betweem 55 and 64)
			}else if(CalculatedPosition> 50 ){
				//turn slightly harder bc close to middle (betweem 55 and 64)
			}else if(CalculatedPosition> 45 ){
				//turn pretty hard bc close to middle (betweem 45 and 64)
			}else{
				//turn really hard right  bc far to the left of the track
			}
			
		}
		

		//on right of the track, need to TURN LEFT
	if(CalculatedPosition>64 ){
		//enter code to turn left
		//TIMER_A2_PWM_DutyCycle(0.1,1);
		//myDelay(25);
					//adding better logic here for different turn amounts
			if(CalculatedPosition< 65 ){
				//turn slightly bc close to middle (betweem 65 and 64
				
			}else if(CalculatedPosition< 70 ){
				//turn slightly bc close to middle (betweem 70 and 64)
			}else if(CalculatedPosition< 75 ){
				//turn slightly harder bc close to middle (betweem 75 and 64)
			}else if(CalculatedPosition< 80 ){
				//turn pretty hard bc close to middle (betweem 80 and 64)
			}else{
				//turn really hard left  bc far to the rightof the track
			}
		
	}
	
	//in center of track, not very likely
	if(CalculatedPosition==64 ){
		//enter code to turn left
		TIMER_A2_PWM_DutyCycle(0.075,1);
		//myDelay(25);
	}

}	
			
