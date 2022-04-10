/*
*This is a copy of the car code without PID as  a backup
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
double PrevServoPosition=0;
double ServoPosition =0;
double RightSpeed = 0;
double PrevRightSpeed = 0;
double LeftSpeed = 0;
double PrevLeftSpeed = 0;
double ForwardCount = 0;
double ForwardCap = 10;
double BreakTime = 0;
double BaseSpeed = 25;
double MaxSpeed = 40;
//double MaxSpeed = BaseSpeed+20;


extern uint16_t line[128];
extern BOOLEAN g_sendData;
//found dif line needs to be a u int 32

void FilterLine(uint16_t SmoothLine[], signed DiffLine[]);
void CalculatePeakLocations(int* leftPeakLoc, int* rightPeakLoc, signed DiffLine[]);
void evaluatePositionANDTurn(int* leftPeakLoc, int* rightPeakLoc, signed DiffLine[]);

void myDelay(int del);
int max(uint16_t SmoothLine[]);
void TurnRightPercent(double percentage);
void TurnLeftPercent(double percentage);
//void AdjustMotors(int* leftPeakLoc, int* rightPeakLoc,int rightPeakFound, int leftPeakFound);
void AdjustMotors();
//void Normalize(void);
//double Average(void);
//double CalculateSD(double mean);
//void Derivative(void);
//void Derivative2(void);


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
    //uart0_init();
	  //uart2_init();
    //uart0_put("Uart Initialized\r\n");
    //init leds for debugging etc
    LED1_Init();
    LED2_Init();
    INIT_Camera();

	  //OLED_Init();	
    //init motors
    init_motors(motor_period);
    //init servos
	  //init_servos(15000);
    init_servos(servo_period);
    EnableInterrupts();
    TIMER_A2_PWM_DutyCycle(0.075,1);
    myDelay(25);
	  //OLED_display_on();
	  //OLED_display_clear();
	  //OLED_display_on();
    
}
int main(){
	  uint16_t SmoothLine[128];
	  signed DiffLine[128];
		//BOOLEAN debug;
	  //char str[200];
    //int i=0;
	  int count = 0;
	  int stop_val = 0;
	  int leftPeakLoc = 0;
	  int rightPeakLoc = 0;
	
		//start pid stuff
		//double err = 0;
		//err = 
		//P
		//double Kp=0;
	
		//I
		//double Ki=0;
	
		//D
		//double Kd=0;
	
		//end pid stuff
	
		//int count = 0;
		//int max_val = 0;
    //debug = 0;
		initialize(); 
		//OLED_display_on();
		//OLED_display_clear();
		//OLED_display_on();
		MotorsForward(25);
	
	

	
	
	
	while(TRUE){
		//LeftMotorForward(30);
		FilterLine(SmoothLine, DiffLine);

		stop_val = max(SmoothLine);
		//sprintf(str,"Max: %i ",stop_val); // start value
		//uart0_put(str);
		
		if(stop_val < 8500){
			count++;
			if(count==50){
				MotorsForward(0);
				break;
			}
		}
		//Normalize();
		//uart0_put("afterNormalize");	
	
		//Derivative();
		//uart0_put("after Derivative");
		
		//Derivative2();
		//uart0_put("after Derivative2");
		//max_val = max();

//		if(debug) {
//			if (g_sendData == TRUE) 
//		{
//			LED1_On();
//			// send the array over uart
//			sprintf(str,"%i\n\r",-1); // start value
//			uart0_put(str);
//			for (i = 0; i < 128; i++) 
//			{
//				sprintf(str,"%i\n\r", DiffLine[i]);
//				uart0_put(str);
//			}
//			sprintf(str,"%i\n\r",-2); // end value
//			uart0_put(str);
//			LED1_Off();
//			g_sendData = FALSE;
//		}
//		}
		//Find left and right edge
		// gets the peak locations dawg 
		CalculatePeakLocations(&leftPeakLoc,&rightPeakLoc,DiffLine);
		//uart0_put("after calculate peaks");
		evaluatePositionANDTurn(&leftPeakLoc,&rightPeakLoc, DiffLine);
		AdjustMotors();
		
//		if(debug){
//			sprintf(str,"Left: %i ",leftPeakLoc); // start value
//			uart0_put(str);
//			sprintf(str,"Right: %i \r\n",rightPeakLoc);
//			uart0_put(str);	
//		}
		//uart0_put("after eval and turn");
		

		//OLED_display_clear();
		//OLED_DisplayCameraData(SmoothLine);
		}
}


void FilterLine(uint16_t SmoothLine[], signed DiffLine[]){
	int L = 0;
	
	for(L=2;L<127;L++){
		SmoothLine[L] = (((line[L-2])+ (line[L-1]) + (line[L]) + (line[L+1]) + (line[L+2]))/5);
	}
	for(L=1;L<127;L++){
		DiffLine[L] = (SmoothLine[L+1] - SmoothLine[L-1])/2;
	}				
		DiffLine[127] = 0;
}

// find the locations of the peaks 
void CalculatePeakLocations(int* leftPeakLoc, int* rightPeakLoc, signed DiffLine[]){
	int idx;
	int TempVal = 999;
 	//tried these starting at 20 but didnt work
	*leftPeakLoc = 64;
	*rightPeakLoc = 64;
	
	//might want to change this to drop first 20 and last 20 instead of 10
	for(idx = 15;idx <120; idx++){
		TempVal = DiffLine[idx];
		//Actual case
		if(TempVal > DiffLine[*leftPeakLoc])
		{
			*leftPeakLoc = idx;
			
		}
	}
	for(idx = 15; idx <120; idx++){
		TempVal = DiffLine[idx];
		//Actual case
		if(TempVal < DiffLine[*rightPeakLoc])
		{
			*rightPeakLoc=idx;
			
			
		}
	}

}

void evaluatePositionANDTurn(int* leftPeakLoc, int* rightPeakLoc, signed DiffLine[]){
	//char streval[200];
	//this is 100/64
	double minimumTurnPercent = 1.5625;
	double ServoRightPercent = 0;
	double ServoLeftPercent = 0;

	int rightPeakFound = 0;//1 for found 0 for not found
	//char str[200];
	int leftPeakFound = 0;//1 for found 0 for not found

	if(DiffLine[*leftPeakLoc] > 700){
			//means we found the left line
			leftPeakFound = 1;
		  LED2_Off();
		  LED2_BLUE_ON();
		
	}
	

	if(DiffLine[*rightPeakLoc] < -700){
			//means we found the right line
			rightPeakFound = 1;
		  LED2_Off();
		  LED2_RED_ON();
	}
	//sprintf(str,"Left: %i ",leftPeakFound); // start value
	//uart0_put(str);
	//sprintf(str,"Left: %i ",DiffLine[*leftPeakLoc]); // start value
	//uart0_put(str);
	//sprintf(str,"Right: %i \r\n",rightPeakFound);
	//uart0_put(str);	
	
  if(rightPeakFound) {
		if (*rightPeakLoc >=63){
			ServoLeftPercent = minimumTurnPercent* (127 - *rightPeakLoc );
			TurnLeftPercent(ServoLeftPercent);
		}else{
			ServoLeftPercent = 100;
			TurnLeftPercent(ServoLeftPercent);
		}
	}
	if(leftPeakFound){
		if (*leftPeakLoc <=63){
			ServoRightPercent = minimumTurnPercent* (*leftPeakLoc);
			TurnRightPercent(ServoRightPercent);
		
		}else{
			ServoRightPercent = 100;
			TurnRightPercent(ServoRightPercent);
		}			
		
	}
	if(leftPeakFound==0 && rightPeakFound==0){
		//TODO
		TurnLeftPercent(0);
		MotorsForward(50);
	}
	//AdjustMotors(leftPeakLoc,rightPeakLoc, leftPeakFound, rightPeakFound);
}	

//void AdjustMotors(int* leftPeakLoc, int* rightPeakLoc, int leftPeakFound, int rightPeakFound){
void AdjustMotors(){
	int loopCounter = 0;
	double helper =0;
	double minimumTurnPercent = 1.5625;
	double ServoRightPercent = 0;
	double ServoLeftPercent = 0;
	PrevRightSpeed = RightSpeed;
	PrevLeftSpeed = LeftSpeed;
	
	//start of addition
//  if(rightPeakFound) {
//		if (*rightPeakLoc >=63){
//			RightSpeed = (MaxSpeed/64)* *rightPeakLoc );
//			TurnLeftPercent(ServoLeftPercent);
//		}else{
//			ServoLeftPercent = 100;
//			TurnLeftPercent(ServoLeftPercent);
//		}
//	}
//	if(leftPeakFound){
//		if (*leftPeakLoc <=63){
//			ServoRightPercent = minimumTurnPercent* (*leftPeakLoc);
//			TurnRightPercent(ServoRightPercent);
//		
//		}else{
//			ServoRightPercent = 100;
//			TurnRightPercent(ServoRightPercent);
//		}			
//		
//	}
	
	//end of addition

	if (ServoPosition>0.07 && ServoPosition<0.08){
		
		RightSpeed = BaseSpeed+5;
		LeftSpeed = BaseSpeed+5;
		RightMotorForward(RightSpeed);
		LeftMotorForward(LeftSpeed);
	}
	
	//means turning left
	if(ServoPosition>0.08){

		
		if(RightSpeed>BaseSpeed +20){
			RightSpeed = BaseSpeed;
			RightMotorReverse(5);
			RightMotorForward(RightSpeed);
		}
		if(LeftSpeed<BaseSpeed +20){
			LeftSpeed = BaseSpeed;
			LeftMotorReverse(5);
			LeftMotorForward(LeftSpeed);
		}
		//*50 because max pwm is 0.1 and wanted to bump speed about 5 in max case
		helper = ServoPosition*50;
		helper = helper/4;
		RightSpeed= RightSpeed + ServoPosition*50;
		if(RightSpeed>BaseSpeed +10){RightSpeed = BaseSpeed+5;}
		if(LeftSpeed<BaseSpeed -10){LeftSpeed = BaseSpeed-10;}
		RightMotorForward(RightSpeed);
		LeftMotorForward(LeftSpeed);
		
	}
	
	//means it is turning right
	if(ServoPosition<0.07){
		if(RightSpeed>BaseSpeed +20){
			RightSpeed = BaseSpeed;
			RightMotorReverse(5);
			RightMotorForward(RightSpeed);
		}
		if(LeftSpeed<BaseSpeed +20){
			LeftSpeed = BaseSpeed;
			LeftMotorReverse(5);
			LeftMotorForward(LeftSpeed);
		}
		
		
		//trying to make 0.05 = 0.1 and flip the rest to make relation easier
		//this is my attempt to scale correctly
		helper = 7.5-ServoPosition*50;
		helper = helper/4;
		RightSpeed= RightSpeed -helper;
		LeftSpeed = LeftSpeed +helper;
		if(RightSpeed<BaseSpeed -10){RightSpeed = BaseSpeed-10;}
		if(LeftSpeed>BaseSpeed +10){LeftSpeed = BaseSpeed+5;}
		RightMotorForward(RightSpeed);
		LeftMotorForward(LeftSpeed);
	}
	
		
	
}


//turn left a percentage given by the user calling
void TurnLeftPercent(double percentage){
	

	double turnDuty =0;
	PrevServoPosition = ServoPosition;
	//make sure its in range
	if(percentage>100){percentage=100;}
	if(percentage<0){percentage=0;}
	
	//if(percentage>=0 && percentage<=100){
	percentage = percentage *.01; //turn percentage into actual value
		//0.025 because thats the difference between left(0.05) and straight (0.075)
	turnDuty = 0.077 + percentage*0.025;
	ServoPosition = turnDuty;
	TIMER_A2_PWM_DutyCycle(turnDuty,1);
	//myDelay(25);
	//}
	
}

void TurnRightPercent(double percentage){
	//make sure its in range
	double turnDuty =0;
	PrevServoPosition = ServoPosition;
	if(percentage>100){percentage=100;}
	if(percentage<0){percentage=0;}
	//if(percentage>=0 && percentage<=100){
		percentage = percentage *.01; //turn percentage into actual value
		//0.025 because thats the difference between straight (0.075) and right(0.1)  
		turnDuty = 0.073 - percentage*0.025;
	
		ServoPosition = turnDuty;
	TIMER_A2_PWM_DutyCycle(turnDuty,1);
	//myDelay(25);
	//}
	
	
}

//double Average(){
//    uint32_t t;
//    double mean;
//    int i;
//    t = 0;
//    for(i=0; i <129; i++){
//        t += SmoothLine[i];
//    }
//    mean = t/129;
//    return(mean);
//}

//int min_left(){
//	int t,i;
//	t = SmoothLine[0];
//	for(i=0; i < 129; i++){
//		if(SmoothLine[i] < t){
//			t = SmoothLine[i];
//		}
//	}
//	return(t);		
//}

int max(uint16_t SmoothLine[]){
	int t,i;
	t = SmoothLine[0];
	for(i=0; i < 129; i++){
		if(SmoothLine[i] > t){
			t = SmoothLine[i];
		}
	}
	return(t);
}

//double CalculateSD(double mean){
//    int i;
//    double sd;
//    for(i=0; i <129; i++){
//        sd += pow((double)(SmoothLine[i] - mean),2);
//    }
//    return(sqrt(sd)/129);
//}
//void Normalize(){
//    double mean,sd;
//    int i;
//    sd = 0.0;
//    mean = Average();
//    sd = CalculateSD(mean);
//    for(i=0; i <129; i++){
//        normalized_trace[i] = (SmoothLine[i] - mean)/sd;
//    } 
//        
//} 

//void Derivative(){
//	//int m = sizeof(normalized_trace)/sizeof(normalized_trace[0]);
//	int i,j;
//	int m = 129;
//	int n = 3;
//	
//	for(i = m; i<132;i++){
//		normalized_trace[i] = 0;
//	}
//	filter[0] = -1;
//	filter[1] = 0;
//	filter[2] = 1;
//	for(i =n;i <132;i++){
//		filter[i] = 0;
//	}
//	
//	for(i=0;i<(n+m);i++){
//		DiffLine[i] = 0;
//		for(j =0; j<m;j++){
//			if(i-j>0){
//				DiffLine[i] = DiffLine[i] + normalized_trace[j]*filter[i-j];
//			
//			}
//		}
//		//might want to take abs value of Diffline[i];
//		//DiffLine[i] = abs(DiffLine[i];
//	}
//	
//	
//	
//}

//void Derivative2(){
//	int L;
//	for(L=2;L<127;L++){
//		DiffLine[L] = ((SmoothLine[L+1] - SmoothLine[L-1])/2);		
//	}
//		DiffLine[126] = (SmoothLine[127] - SmoothLine[125])/2;
//		DiffLine[127] = (SmoothLine[127] - SmoothLine[126]);
//	
//}

			
