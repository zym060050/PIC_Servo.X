/*
 * File:   PIC_Encoder.c
 * Author: YWong3
 *
 * Created on November 25, 2015, 10:43 AM
 */


#include <xc.h>
#include "PIC_Servo.h"
#include <plib/timers.h>

#define USE_AND_MASKS
unsigned char Timer0Config, Timer1Config;
unsigned int oldTimer0, oldTimer1;
void initEncoders(void){
    Timer0Config = TIMER_INT_OFF & T0_16BIT & T0_SOURCE_EXT & T0_PS_1_1 ;
    Timer1Config = TIMER_INT_OFF & T1_16BIT_RW & T1_SOURCE_EXT & T1_OSC1EN_OFF & T3_SOURCE_EXT & T1_PS_1_1 & T1_SYNC_EXT_OFF;
    OpenTimer0(Timer0Config);
    OpenTimer1(Timer1Config);
    oldTimer0=0;
    oldTimer1=0;
}

void updatePosition(unsigned int *currentPos, unsigned char direction, unsigned char source) {
    unsigned int count;
    unsigned int count_move;
    if (source==0){
        count = ReadTimer0();
        count_move = count - oldTimer0;
    }
    else{
        count = ReadTimer1();
        count_move = count - oldTimer1;
    }
        
    if(direction){ //forward
        (*currentPos)=(*currentPos)+count_move;
    }
    else{
        (*currentPos)=(*currentPos)-count_move;
    }
    
    if (source == 0) {
        oldTimer0 = count;
    } else {
        oldTimer1 = count;
    }
   
    return;
}
