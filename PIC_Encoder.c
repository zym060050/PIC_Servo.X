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
#define CMD_A 0x01
#define CMD_B 0x02
#define MOVE_ABS 0x01
#define MOVE_INC 0x02
#define CMD_A_MOVE_FWD 0x2a
#define CMD_A_MOVE_REV 0x2b
#define CMD_A_STOP 0x2c
#define CMD_B_MOVE_FWD 0x3a
#define CMD_B_MOVE_REV 0x3b
#define CMD_B_STOP 0x3c

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
    //currentPos should be 0-->MAX (needs to check and should be always the same)
    //Start the x y axis at extreme ends before moving
    //with this setup, moving to absolute position is possible
    unsigned int count;
    unsigned int count_move;
    if (source==0){
        count = ReadTimer0();
        count_move = count - oldTimer0; //oldTimer0 is actually old counts of Timer0 ext input
    }
    else{
        count = ReadTimer1();
        //This step will ensure that even if we read too slowly at 10ms interrupt, the counts are correct
        count_move = count - oldTimer1; //oldTimer1 is actually old counts of Timer1 ext input
    }
        
    if(direction){ //forward
        (*currentPos)=(*currentPos)+count_move;
        //Might need to implement a max
    }
    else{
        (*currentPos)=(*currentPos)-count_move;
        //might need to implement a min
    }
    
    if (source == 0) {
        oldTimer0 = count; //moves current count to old count
    } else {
        oldTimer1 = count;
    }
   
    return;
}

void stopMotor(unsigned char CMD){
    if(CMD==CMD_A_STOP){
        M_A1 = DISABLE_ACTIVE_LOW;
        M_A2 = DISABLE_ACTIVE_LOW;
    }
    else if(CMD==CMD_B_STOP){
        M_B1 = DISABLE_ACTIVE_LOW;
        M_B2 = DISABLE_ACTIVE_LOW;
    }
    else {
        M_A1 = DISABLE_ACTIVE_LOW;
        M_A2 = DISABLE_ACTIVE_LOW;
        M_B1 = DISABLE_ACTIVE_LOW;
        M_B2 = DISABLE_ACTIVE_LOW;
    }
}

void moveMotor(unsigned char CMD){
    
}