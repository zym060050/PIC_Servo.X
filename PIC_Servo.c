/* 
 * File:   PIC_Servo.c
 * Author: Administrator
 *
 * Created on October 19, 2015, 9:19 PM
 */
#include <stdio.h>
#include <stdlib.h>
#include "PIC_Servo.h"

//This session is must if use Tinybld as it creates a GoTo instruction at start,
void interrupt Do_goto(void)
{
}
//to avoid overwriting the bootloader

volatile unsigned long time;

#define UART_BUFFER_SIZE		6
#define UART_BUFFER_DATA_SIZE 	PACKET_LENGTH
static volatile unsigned char UART_Buffer_Index_Count = 0;
static volatile unsigned char UART_Buffer_Data_Count = 0;
static volatile unsigned char  UARTBuffer_RX[UART_BUFFER_SIZE][UART_BUFFER_DATA_SIZE];

static volatile unsigned char UART_Buffer_Process_Index = 0;
unsigned char CMD_Buffer[UART_BUFFER_DATA_SIZE];

// function declare
static void Process_Uart_Rx_Buffer(void);

// low_priority interrupt handler
void interrupt low_priority interrupt_handler(void)          
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
    
    //timer3 interrupt
    if (TMR3IE && TMR3IF)
    {
        time++;
        TMR3L = T3_START_COUNT_LO;
        TMR3H = T3_START_COUNT_HI;
        TMR3IF=0;
    }
}

void main (void)
{  
    Initialize();
    
    serial_Putstr("Hi\n",3);

    __delay_ms(19); //max value
   
    while (1)
    {
        Process_Uart_Rx_Buffer();
    }
}

static void Process_Uart_Rx_Buffer(void)
{
    if(UART_Buffer_Process_Index != UART_Buffer_Index_Count)
    {
        unsigned char i = 0;
        unsigned char address = 0;
        
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
                        PIC_Motor_Control(MOTOR_A, MOTOR_CONTROL_FW, CMD_Buffer[CMD_POS_DATA1]);
                        break;
                    case CMD_MOTOR_A_BW:
                        PIC_Motor_Control(MOTOR_A, MOTOR_CONTROL_BW, CMD_Buffer[CMD_POS_DATA1]);
                        break;
                    case CMD_MOTOR_A_STOP:
                        PIC_Motor_Control(MOTOR_A, MOTOR_CONTROL_STOP, CMD_Buffer[CMD_POS_DATA1]);
                        break;
                    case CMD_MOTOR_B_FW:
                        PIC_Motor_Control(MOTOR_B, MOTOR_CONTROL_FW, CMD_Buffer[CMD_POS_DATA1]);
                        break;
                    case CMD_MOTOR_B_BW:
                        PIC_Motor_Control(MOTOR_B, MOTOR_CONTROL_BW, CMD_Buffer[CMD_POS_DATA1]);
                        break;
                    case CMD_MOTOR_B_STOP:
                        PIC_Motor_Control(MOTOR_B, MOTOR_CONTROL_STOP, CMD_Buffer[CMD_POS_DATA1]);
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
