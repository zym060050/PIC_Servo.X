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

volatile unsigned long time_10ms;

#define UART_BUFFER_SIZE		6
#define UART_BUFFER_DATA_SIZE 	PACKET_LENGTH
static volatile unsigned char UART_Buffer_Index_Count = 0;
static volatile unsigned char UART_Buffer_Data_Count = 0;
static volatile unsigned char  UARTBuffer_RX[UART_BUFFER_SIZE][UART_BUFFER_DATA_SIZE];

static volatile unsigned char UART_Buffer_Process_Index = 0;
unsigned char CMD_Buffer[UART_BUFFER_DATA_SIZE];

unsigned char lastATach0[4] = 0;
unsigned char lastATach1[4] = 0;
unsigned char ATach0 = 0;
unsigned char ATach1 = 0;

unsigned char currentDirection = 0;

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
        time_10ms++;
        TMR3L = T3_START_COUNT_LO;
        TMR3H = T3_START_COUNT_HI;
        TMR3IF=0;
    }
    
    if (INT1IF || INT2IF) {
        unsigned char currentReading = MA_Tacho0 | (MA_Tacho1 << 1);
        if (lastATach0[0] == 0 && lastATach0[1] == 0) {
            lastATach0[0] = currentReading;
        } else {
            switch (lastATach0[0]) {
                case 0:
                    if (currentReading == 1) {
                        MotorA_Position--;
                    } else if (currentReading == 2) {
                        MotorA_Position++;
                    }
                    break;
                case 1:
                    if (currentReading == 3) {
                        MotorA_Position--;
                    } else if (currentReading == 0) {
                        MotorA_Position++;
                    }
                    break;
                case 2:
                    if (currentReading == 0) {
                        MotorA_Position--;
                    } else if (currentReading == 3) {
                        MotorA_Position++;
                    }
                    break;
                case 3:
                    if (currentReading == 2) {
                        MotorA_Position--;
                    } else if (currentReading == 1) {
                        MotorA_Position++;
                    }
                    break;

                default:
                    break;
            }
            lastATach0[0] = currentReading;
        }
        if (currentDirection == CMD_MOTOR_A_FW) {
            if (MotorA_Position > motorATargetPos) {
                //moving FW
                PIC_Motor_Control(MOTOR_A, MOTOR_CONTROL_STOP, 0);
            }
        } else if (currentDirection == CMD_MOTOR_A_BW) {
            if (MotorA_Position < motorATargetPos) {
                //moving BW
                PIC_Motor_Control(MOTOR_A, MOTOR_CONTROL_STOP, 0);
            }
        }
        
        INT1IF = 0;
        INT2IF = 0;
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
        //PIC_Motor_Control(MOTOR_A, MOTOR_CONTROL_FW, CMD_Buffer[CMD_POS_DATA1]);
        
#endif
    }
}

static void Process_Uart_Rx_Buffer(void)
{
    if(UART_Buffer_Process_Index != UART_Buffer_Index_Count)
    {
        unsigned char i = 0;
        unsigned char address = 0;
        int aInt=0;
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
                        currentDirection=CMD_MOTOR_A_FW;
                        break;
                    case CMD_MOTOR_A_BW:
                        PIC_Motor_Control(MOTOR_A, MOTOR_CONTROL_BW, (CMD_Buffer[CMD_POS_DATA1] | (CMD_Buffer[CMD_POS_DATA2]<<8)));
                        currentDirection=CMD_MOTOR_A_BW;
                        break;
                    case CMD_MOTOR_A_STOP:
                        PIC_Motor_Control(MOTOR_A, MOTOR_CONTROL_STOP, (CMD_Buffer[CMD_POS_DATA1] | (CMD_Buffer[CMD_POS_DATA2]<<8)));
                        currentDirection=CMD_MOTOR_A_STOP;
                        break;
                    case CMD_MOTOR_B_FW:
                        PIC_Motor_Control(MOTOR_B, MOTOR_CONTROL_FW, CMD_Buffer[CMD_POS_DATA1]);
                        currentDirection=CMD_MOTOR_B_FW;
                        break;
                    case CMD_MOTOR_B_BW:
                        PIC_Motor_Control(MOTOR_B, MOTOR_CONTROL_BW, CMD_Buffer[CMD_POS_DATA1]);
                        currentDirection=CMD_MOTOR_B_BW;
                        break;
                    case CMD_MOTOR_B_STOP:
                        PIC_Motor_Control(MOTOR_B, MOTOR_CONTROL_STOP, CMD_Buffer[CMD_POS_DATA1]);
                        currentDirection=CMD_MOTOR_B_STOP;
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
                        //aInt = MotorA_Position;
                        str[0] = MotorA_Position & 0xff;
                        str[1] = (MotorA_Position >> 8) & 0xff;
                        //sprintf(str, "%d", MotorA_Position);
                        serial_Putstr(str,2);
                        //RESET();
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
