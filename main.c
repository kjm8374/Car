/**
 * main.c
 */
#include "msp.h"
#include "oled.h"

#include <stdio.h>
#include <string.h>
extern  unsigned char OLED_clr_data[1024];
extern unsigned char OLED_TEXT_ARR[1024];
extern unsigned char OLED_GRAPH_ARR[1024];
uint16_t lineData[128];
//void msdelay(int delay)
//{
//    int i,j;
//    for(i=0;i<delay;i++)
//        for(j=0;j<16000;j++);
//}

//int main(void)
//{
//	int i;
//	int delta;
//	OLED_Init();
//	OLED_display_on();
//	OLED_display_clear();
//	OLED_display_on();
//    // write "Hello World" on line 1, column 1
//    // write "How are you?" on line 2, column 2
//    // write "Goodbye" on line 3, column 3
//	// (HINT: Use OLED_draw_line)
//	OLED_draw_line(1,1,"Hello World");
//	OLED_draw_line(2,3,"How are you?");
//	OLED_draw_line(3,3,"Goodbye");
//    
//	// create some fake 14-bit camera line data 0 - >2^14 in a 128 short array that utilizes all 14 bits

//	for (i = 0; i < 128; i++)
//		//lineData[i] = ??;
//    lineData[i] = 128 * i ;

//	while(1)
//	{
//			
//			OLED_write_display(OLED_TEXT_ARR);
//			msdelay(500);
//			OLED_DisplayCameraData(lineData);
//			msdelay(500);

//	}
//}
