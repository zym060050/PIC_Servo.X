/* 
 * File:   PIC_Servo.c
 * Author: Administrator
 *
 * Created on October 19, 2015, 9:19 PM
 */
#include <stdio.h>
#include <stdlib.h>
#include "PIC_Servo.h"

////This session is must if use Tinybld as it creates a GoTo instruction at start,
//void interrupt Do_goto(void)
//{
//}
////to avoid overwriting the bootloader

//Counter
volatile unsigned long time_10ms;
//UART Buffer
#define UART_BUFFER_SIZE		6
#define UART_BUFFER_DATA_SIZE 	PACKET_LENGTH
static volatile unsigned char UART_Buffer_Index_Count = 0;
static volatile unsigned char UART_Buffer_Data_Count = 0;
static volatile unsigned char  UARTBuffer_RX[UART_BUFFER_SIZE][UART_BUFFER_DATA_SIZE];
//Command Buffer
static volatile unsigned char UART_Buffer_Process_Index = 0;
unsigned char CMD_Buffer[UART_BUFFER_DATA_SIZE];

// function declare
static void Process_Uart_Rx_Buffer(void);

// high_priority interrupt handler
void high_priority interrupt high_priority_interrupt_handler(void)
{
    //uart interrupt
    if (RCIE && RCIF)
    {
        UARTBuffer_RX[UART_Buffer_Index_Count][UART_Buffer_Data_Count++] = RCREG;
        if((UART_Buffer_Data_Count == 1) && (UARTBuffer_RX[UART_Buffer_Index_Count][0] != SOH))
        {
            UART_Buffer_Data_Count = 0;
        }
        else
        {
            if (UART_Buffer_Data_Count == UART_BUFFER_DATA_SIZE)
            {
                UART_Buffer_Index_Count++;
                if(UART_Buffer_Index_Count == UART_BUFFER_SIZE)
                {
                    UART_Buffer_Index_Count =0;
                }
                UART_Buffer_Data_Count = 0;
            }
        }
    }
    
    //INT interrupt
    /*Motor A Tacho 0*/
    if(INT2IE && INT2IF)
    {
        if(MA_Tacho1)
        {
            //FW
            MotorA_Position++;
#ifndef ENABLE_PID_CONTROL
            //speed control
            if(MotorA_Position<motorATargetPos)
            {
                PIC_Motor_Speed_Configure(MOTOR_A, motorATargetPos-MotorA_Position);
            }
            //stop check
            if (MotorA_Position >= motorATargetPos)
            {
                //this checking is to avoid the situation such that when motor is moving and motor move to instruction is given to move to reverse direction,
                //without this checking, motor still have the Momentum after the move to instruction was given and next interrupt comes in cause motor to stop here.
                //this applies to motor control without PID.
                if ((MotorA_Position-motorATargetPos) < MOTOR_STOP_CHECK_THRESHOLD)
                {
                    //moving FW stop
                    PIC_Motor_Control(MOTOR_A, MOTOR_CONTROL_STOP, 0);
                }
            }
#endif
        }
        else
        {
            //BW
            MotorA_Position--;
#ifndef ENABLE_PID_CONTROL
            //speed control
            if(MotorA_Position>motorATargetPos)
            {
                PIC_Motor_Speed_Configure(MOTOR_A, MotorA_Position-motorATargetPos);
            }
            //stop check
            if (MotorA_Position <= motorATargetPos)
            {
                //this checking is to avoid the situation such that when motor is moving and motor move to instruction is given to move to reverse direction,
                //without this checking, motor still have the Momentum after the move to instruction was given and next interrupt comes in cause motor to stop here.
                //this applies to motor control without PID.
                if ((motorATargetPos-MotorA_Position) < MOTOR_STOP_CHECK_THRESHOLD)
                {
                    //moving BW stop
                    PIC_Motor_Control(MOTOR_A, MOTOR_CONTROL_STOP, 0);
                }
            }
#endif
        }
        INT2IF = 0;
    }
    /*Motor B Tacho 0*/
    if(INT1IE && INT1IF)
    {
        if(MB_Tacho1)
        {
            //FW
            MotorB_Position++;
#ifndef ENABLE_PID_CONTROL
            //speed control
            if(MotorB_Position<motorBTargetPos)
            {
                PIC_Motor_Speed_Configure(MOTOR_B, motorBTargetPos-MotorB_Position);
            }
            //stop check
            if (MotorB_Position >= motorBTargetPos)
            {
                //this checking is to avoid the situation such that when motor is moving and motor move to instruction is given to move to reverse direction,
                //without this checking, motor still have the Momentum after the move to instruction was given and next interrupt comes in cause motor to stop here.
                //this applies to motor control without PID.
                if ((MotorB_Position-motorBTargetPos) < MOTOR_STOP_CHECK_THRESHOLD)
                {
                    //moving FW stop
                    PIC_Motor_Control(MOTOR_B, MOTOR_CONTROL_STOP, 0);
                }
            }
#endif
        }
        else
        {
            //BW
            MotorB_Position--;
#ifndef ENABLE_PID_CONTROL
            //speed control
            if(MotorB_Position>motorBTargetPos)
            {
                PIC_Motor_Speed_Configure(MOTOR_B, MotorB_Position-motorBTargetPos);
            }
            //stop check
            if (MotorB_Position <= motorBTargetPos)
            {
                //this checking is to avoid the situation such that when motor is moving and motor move to instruction is given to move to reverse direction,
                //without this checking, motor still have the Momentum after the move to instruction was given and next interrupt comes in cause motor to stop here.
                //this applies to motor control without PID.
                if ((motorBTargetPos-MotorB_Position) < MOTOR_STOP_CHECK_THRESHOLD)
                {
                    //moving BW stop
                    PIC_Motor_Control(MOTOR_B, MOTOR_CONTROL_STOP, 0);
                }
            }
#endif
        }
        INT1IF = 0;
    }
}

// low_priority interrupt handler
void low_priority interrupt low_priority_interrupt_handler(void)
{
    //timer3 interrupt
    if (TMR3IE && TMR3IF)
    {
        time_10ms++;
        TMR3L = T3_START_COUNT_LO;
        TMR3H = T3_START_COUNT_HI;
#ifdef ENABLE_PID_CONTROL
        PIC_Motor_PID_Loop();
#endif
        TMR3IF=0;
    }
}

void wait_for_10ms(unsigned long no_of_10ms)
{
    time_10ms = 0;
    while(time_10ms<no_of_10ms);
}

void main (void)
{  
    Initialize();
    
    __delay_ms(19); //max value
    
    while (1)
    {
#ifdef PCB_BOARD_VERIFY_LED
        LED_STATUS = LED_ON;
        wait_for_10ms(50);
        LED_STATUS = LED_OFF;
        wait_for_10ms(50);
#else
        Process_Uart_Rx_Buffer();
#endif
    }
}

static void Process_Uart_Rx_Buffer(void)
{
    if(UART_Buffer_Process_Index != UART_Buffer_Index_Count)
    {
        unsigned char i = 0;
        unsigned char address = 0;
        char str[15];
        
        // get data
        for(i = 0; i < UART_BUFFER_DATA_SIZE; i++)
		{
			CMD_Buffer[i] = UARTBuffer_RX[UART_Buffer_Process_Index][i];
			UARTBuffer_RX[UART_Buffer_Process_Index][i] = 0;
		}
		UART_Buffer_Process_Index++;
		if(UART_Buffer_Process_Index == UART_BUFFER_SIZE)
        {
			UART_Buffer_Process_Index = 0;
        }
        
        // read address from DIP switch
        address = BOARD_ID;
        // check CRC
        if (CheckCRC(CMD_Buffer,UART_BUFFER_DATA_SIZE) == OK)
        {
            // Check Address ...
            if ((CMD_Buffer[CMD_POS_ADDR] == address)||(CMD_Buffer[CMD_POS_ADDR] == GLOBAL_ADDRESS))
            {
                // Execute Command ...
                switch (CMD_Buffer[CMD_POS_CMD])
                {
                    case CMD_MOTOR_A_FW:
                        PIC_Motor_Control(MOTOR_A, MOTOR_CONTROL_FW, (CMD_Buffer[CMD_POS_DATA1] | (CMD_Buffer[CMD_POS_DATA2]<<8)));
                        break;
                    case CMD_MOTOR_A_BW:
                        PIC_Motor_Control(MOTOR_A, MOTOR_CONTROL_BW, (CMD_Buffer[CMD_POS_DATA1] | (CMD_Buffer[CMD_POS_DATA2]<<8)));
                        break;
                    case CMD_MOTOR_A_STOP:
                        PIC_Motor_Control(MOTOR_A, MOTOR_CONTROL_STOP, (CMD_Buffer[CMD_POS_DATA1] | (CMD_Buffer[CMD_POS_DATA2]<<8)));
                        break;
                    case CMD_MOTOR_B_FW:
                        PIC_Motor_Control(MOTOR_B, MOTOR_CONTROL_FW, (CMD_Buffer[CMD_POS_DATA1] | (CMD_Buffer[CMD_POS_DATA2]<<8)));
                        break;
                    case CMD_MOTOR_B_BW:
                        PIC_Motor_Control(MOTOR_B, MOTOR_CONTROL_BW, (CMD_Buffer[CMD_POS_DATA1] | (CMD_Buffer[CMD_POS_DATA2]<<8)));
                        break;
                    case CMD_MOTOR_B_STOP:
                        PIC_Motor_Control(MOTOR_B, MOTOR_CONTROL_STOP, (CMD_Buffer[CMD_POS_DATA1] | (CMD_Buffer[CMD_POS_DATA2]<<8)));
                        break;
                    case CMD_MOTOR_READ_COUNT:
                        str[0] = MotorA_Position & 0xff;
                        str[1] = (MotorA_Position >> 8) & 0xff;
                        str[2] = (MotorA_Position >> 16) & 0xff;
                        str[3] = (MotorA_Position >> 24) & 0xff;
                        str[4] = MotorB_Position & 0xff;
                        str[5] = (MotorB_Position >> 8) & 0xff;
                        str[6] = (MotorB_Position >> 16) & 0xff;
                        str[7] = (MotorB_Position >> 24) & 0xff;
                        serial_Putstr(str,8);
                        break;
                    case CMD_MOTOR_A_MOVE_TO:
                        PIC_Motor_Move_To_Position(MOTOR_A, (CMD_Buffer[CMD_POS_DATA1] | (CMD_Buffer[CMD_POS_DATA2]<<8)));
                        break;
                    case CMD_MOTOR_B_MOVE_TO:
                        PIC_Motor_Move_To_Position(MOTOR_B, (CMD_Buffer[CMD_POS_DATA1] | (CMD_Buffer[CMD_POS_DATA2]<<8)));
                        break;
                    #ifdef ENABLE_PID_CONTROL
                    case CMD_PID_OUTPUT_LIMIT:
                        if (CMD_Buffer[CMD_POS_DATA1]>MOTOR_MAX_SPEED)
                            PID_OUTPUT_LIMIT = MOTOR_MAX_SPEED;
                        else
                            PID_OUTPUT_LIMIT = CMD_Buffer[CMD_POS_DATA1];
                        break;
                    #endif
                    case CMD_MOTOR_AB_MOVE_TO:
                        //TODO:
                        /*Will Add feedback implementation later when the motor(A and B) moved to the target position.*/
                        //Control motor:
                        MA_Status=0;
                        MB_Status=0;
                        PIC_Motor_Move_To_Position(MOTOR_A, (CMD_Buffer[CMD_POS_DATA1] | (CMD_Buffer[CMD_POS_DATA2]<<8)));
                        PIC_Motor_Move_To_Position(MOTOR_B, (CMD_Buffer[CMD_POS_DATA3] | (CMD_Buffer[CMD_POS_DATA4]<<8)));
                        break;
                    #ifndef NEW_PCB_BOARD
                    case CMD_CONTROL_LED:
                        if(CMD_Buffer[CMD_POS_DATA1])
                        {
                            LED_STATUS = LED_ON;
                        }
                        else
                        {
                            LED_STATUS = LED_OFF;
                        }
                        break;
                    #endif
                    case CMD_RESET_MAIN_MCU:
                        //house-keeping here
                        RESET();
                        break;
                    default:
                        break;
                }
            }
        }
        
        // clear command buffer
        for(i = 0; i < UART_BUFFER_DATA_SIZE; i++)
		{
            CMD_Buffer[i] = 0;
        }
    }
}
