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
#include "oled.h"
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
#include <math.h>
#include <stdlib.h>


double normalized_trace[128];
double filter[131];


extern uint16_t line[128];
extern BOOLEAN g_sendData;
uint16_t SmoothLine[128];
//found dif line needs to be a u int 32
uint32_t DiffLine[128];
BOOLEAN debug;

void CalculatePeakLocations(int* leftPeakLoc, int* rightPeakLoc);
void evaluatePositionANDTurn(int* leftPeakLoc, int* rightPeakLoc, int* CalculatedPosition);
void FilterLine(void);
void myDelay(int del);
int max(void);
void TurnRightPercent(double percentage);
void TurnLeftPercent(double percentage);
void Normalize(void);
double Average(void);
double CalculateSD(double mean);
void Derivative(void);
void Derivative2(void);


//should be delay in ms (del)
void myDelay(int del){
	volatile int i;
	for (i=0; i<del*50000; i++){
		;// Do nothing
	}
}

void initialize() {
    
    uint16_t motor_period = 48000000/10000;
    uint32_t servo_period = 3000000/50;
    DisableInterrupts();
    uart0_init();
    //uart0_put("Uart Initialized\r\n");
    //init leds for debugging etc
    LED1_Init();
    LED2_Init();
    INIT_Camera();
    //init motors
    init_motors(motor_period);
    //init servos
    init_servos(servo_period);
    EnableInterrupts();
    TIMER_A2_PWM_DutyCycle(0.075,1);
    myDelay(25);
    
}
int main(){
	  char str[200];
    int i=0;
	  int leftPeakLoc = 0;
	  int rightPeakLoc = 0;
	  int CalculatedPosition = 0;
		//int count = 0;
		//int max_val = 0;
    debug = 0;
		initialize(); 
	//OLED_display_on();
	//OLED_display_clear();
	//OLED_display_on();

	while(TRUE){
		
		FilterLine();

		
		//Normalize();
		//uart0_put("afterNormalize");	
	
		//Derivative();
		//uart0_put("after Derivative");
		
		//Derivative2();
		//uart0_put("after Derivative2");
		//max_val = max();

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
		CalculatePeakLocations(&leftPeakLoc,&rightPeakLoc);
		//uart0_put("after calculate peaks");
		evaluatePositionANDTurn(&leftPeakLoc,&rightPeakLoc, &CalculatedPosition);
		if(!debug){
			sprintf(str,"Left: %i ",leftPeakLoc); // start value
			uart0_put(str);
			sprintf(str,"Right: %i ",rightPeakLoc);
			uart0_put(str);
			sprintf(str,"Calculated position: %i\r\n",CalculatedPosition);
			uart0_put(str);			
		}
		//uart0_put("after eval and turn");
		
		//OLED_display_clear();
		//OLED_DisplayCameraData(line);

		MotorsForward(25);
		}
}


void FilterLine(){
	int L = 0;
	
	
	for(L=2;L<127;L++){
		SmoothLine[L] = (((line[L-2])+ (line[L-1]) + (line[L]) + (line[L+1]) + (line[L+2]))/5);
		SmoothLine[L+1] = (((line[L-1])+ (line[L]) + (line[L+1]) + (line[L+2]) + (line[L+3]))/5);	
		DiffLine[L] = abs((SmoothLine[L+1] - SmoothLine[L-1])/2);		
	}
		DiffLine[126] = abs((SmoothLine[127] - SmoothLine[125])/2);
		DiffLine[127] = abs(SmoothLine[127] - SmoothLine[126]);
}

// find the locations of the peaks 
void CalculatePeakLocations(int* leftPeakLoc, int* rightPeakLoc){
	int idx;
	int TempVal = 999;
	//tried these starting at 20 but didnt work
	*leftPeakLoc = 64;
	*rightPeakLoc = 64;
	
	//might want to change this to drop first 20 and last 20 instead of 10
	for(idx = 10;idx <65; idx++){
		TempVal = DiffLine[idx];
		//Actual case
		if(TempVal > DiffLine[*leftPeakLoc])
		{
			*leftPeakLoc = idx;
		}
	}
	for(idx = 65; idx <120; idx++){
		TempVal = DiffLine[idx];
		//Actual case
		if(TempVal > DiffLine[*rightPeakLoc])
		{
			*rightPeakLoc=idx;
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

void evaluatePositionANDTurn(int* leftPeakLoc, int* rightPeakLoc, int* CalculatedPosition){
	//char streval[200];
	int i =0;
	int rightPeakFound = 0;//1 for found 0 for not found
	int leftPeakFound = 0;//1 for found 0 for not found
	for(i=10; i<65;i++){
		if(DiffLine[i] > 1000){
			//means we found the left line
			//LeftFound=1;
			leftPeakFound = 1;
		}
	}
	
		for(i=64; i<118;i++){
		if(DiffLine[i] > 1000){
			//means we found the right line
			//RightFound=1;
			rightPeakFound = 1;
		}
	}
		
	
	if(leftPeakFound==1 && rightPeakFound == 1){
		*CalculatedPosition = *rightPeakLoc-*leftPeakLoc;
		uart0_put("found both lines/n");
	
	}
	if(leftPeakFound==1 && rightPeakFound == 0){
			//means that we found both lines
			*CalculatedPosition = *leftPeakLoc;
		}
	 if(leftPeakFound==0 && rightPeakFound == 1){
			//cant find left but found the right
			//verify its actually an edge
			*CalculatedPosition = *rightPeakLoc;
		}

	//sprintf(streval,"%i\n\r",CalculatedPosition); // start value
	//uart0_put(streval);
	//on left of the track, need to TURN RIGHT
	if(*CalculatedPosition< 64 )
		{
		//entercode to turn right
		//TIMER_A2_PWM_DutyCycle(0.05,1);
		//myDelay(25);
		//myDelay(25);
			//adding better logic here for different turn amounts
			if(*CalculatedPosition> 60 ){
				TurnRightPercent(100);
				myDelay(25);
				//turn slightly bc close to middle (betweem 60 and 64
				
			}else if(*CalculatedPosition> 50 ){
				TurnRightPercent(90);
				myDelay(25);
				//turn slightly bc close to middle (betweem 55 and 64)
			}else if(*CalculatedPosition> 40 ){
				TurnRightPercent(75);
				myDelay(25);
				//turn slightly harder bc close to middle (betweem 55 and 64)
			}else if(*CalculatedPosition> 30 ){
				TurnRightPercent(60);
				myDelay(25);
				//turn pretty hard bc close to middle (betweem 45 and 64)
			}else{
				TurnRightPercent(0);
				myDelay(25);
				//turn really hard right  bc far to the left of the track
			}	
		}
		

		//on right of the track, need to TURN LEFT
	else if(*CalculatedPosition>64 ){
		//enter code to turn left
		//TIMER_A2_PWM_DutyCycle(0.1,1);
		myDelay(25);
		//myDelay(25);
					//adding better logic here for different turn amounts
			if(*CalculatedPosition < 70 ){
				TurnLeftPercent(100);
				myDelay(25);
				//turn slightly bc close to middle (betweem 65 and 64
			}else if(*CalculatedPosition< 80 ){
				TurnLeftPercent(90);
				myDelay(25);
				//turn slightly bc close to middle (betweem 70 and 64)
			}else if(*CalculatedPosition< 90 ){
				TurnLeftPercent(80);
				myDelay(25);
				//turn slightly harder bc close to middle (betweem 75 and 64)
			}else if(*CalculatedPosition< 100 ){
				TurnLeftPercent(70);
				myDelay(25);
				//turn pretty hard bc close to middle (betweem 80 and 64)
			}else{
				TurnLeftPercent(0);
				myDelay(25);
				//turn really hard left  bc far to the rightof the track
			}
		
	}
	else{
		TIMER_A2_PWM_DutyCycle(0.075,1);
		myDelay(25);
	}
	
	//in center of track, not very likely
	if(*CalculatedPosition==64 ){
		TIMER_A2_PWM_DutyCycle(0.075,1);
		//myDelay(25);
	}

}	
//turn left a percentage given by the user calling
void TurnLeftPercent(double percentage){
	double turnDuty =0;
	//make sure its in range
	if(percentage>=0 && percentage<=100){
	percentage = percentage *.01; //turn percentage into actual value
		//0.025 because thats the difference between left(0.05) and straight (0.075)
	turnDuty = 0.075 + percentage*0.025;
	
	TIMER_A2_PWM_DutyCycle(turnDuty,1);
	}
	
}

void TurnRightPercent(double percentage){
	//make sure its in range
	double turnDuty =0;
	if(percentage>=0 && percentage<=100){
		percentage = percentage *.01; //turn percentage into actual value
		//0.025 because thats the difference between straight (0.075) and right(0.1)  
		turnDuty = 0.075 - percentage*0.025;
	
	TIMER_A2_PWM_DutyCycle(turnDuty,1);
	}
	
	
}

double Average(){
    uint32_t t;
    double mean;
    int i;
    t = 0;
    for(i=0; i <129; i++){
        t += SmoothLine[i];
    }
    mean = t/129;
    return(mean);
}

int max(){
	int t,i;
	t = SmoothLine[0];
	for(i=0; i < 129; i++){
		if(SmoothLine[i] > t){
			t = SmoothLine[i];
		}
	}
	return(t);
}

double CalculateSD(double mean){
    int i;
    double sd;
    for(i=0; i <129; i++){
        sd += pow((double)(SmoothLine[i] - mean),2);
    }
    return(sqrt(sd)/129);
}
void Normalize(){
    double mean,sd;
    int i;
    sd = 0.0;
    mean = Average();
    sd = CalculateSD(mean);
    for(i=0; i <129; i++){
        normalized_trace[i] = (SmoothLine[i] - mean)/sd;
    } 
        
} 

void Derivative(){
	//int m = sizeof(normalized_trace)/sizeof(normalized_trace[0]);
	int i,j;
	int m = 129;
	int n = 3;
	
	for(i = m; i<132;i++){
		normalized_trace[i] = 0;
	}
	filter[0] = -1;
	filter[1] = 0;
	filter[2] = 1;
	for(i =n;i <132;i++){
		filter[i] = 0;
	}
	
	for(i=0;i<(n+m);i++){
		DiffLine[i] = 0;
		for(j =0; j<m;j++){
			if(i-j>0){
				DiffLine[i] = DiffLine[i] + normalized_trace[j]*filter[i-j];
			
			}
		}
		//might want to take abs value of Diffline[i];
		//DiffLine[i] = abs(DiffLine[i];
	}
	
	
	
}

void Derivative2(){
	int L;
	for(L=2;L<127;L++){
		DiffLine[L] = ((SmoothLine[L+1] - SmoothLine[L-1])/2);		
	}
		DiffLine[126] = (SmoothLine[127] - SmoothLine[125])/2;
		DiffLine[127] = (SmoothLine[127] - SmoothLine[126]);
	
}

			
