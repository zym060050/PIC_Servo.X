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

unsigned char state;
volatile unsigned long time;

// low_priority interrupt handler
void interrupt low_priority interrupt_handler(void)          
{
    //interrupt_handler
    
    //timer3_int
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
    unsigned char data[PACKET_LENGTH], dataLength, address, CmdId, i;
      
    Initialize();
    
    serial_Putstr("Hi\n",3);

    __delay_ms(19); //max value
   
    while (1)
    {
        switch (state)
        {
            // Wait for start of a packet
            case IDLE:
#ifdef SERIAL_ECHO_TEST
                if (RCIF == 1)
                {
                    data[CMD_POS_SOH] = RCREG;
                    serial_Putch(data[CMD_POS_SOH]);
                }
#else
                if (RCIF == 1)
                {
                    if (RCREG == SOH)
                    {
                        // Store received byte to check CRC
                        data[CMD_POS_SOH] = SOH;
                        // reset timer to check the packet time out
                        time = 0;
                        // reset data length counter
                        dataLength = 1;
                        // Go to the next state: Get the remaining bytes of the packet
                        state = GET_PACKET_DATA;
                    }
                }
#endif
                break;
                
            // Get the remaining bytes of the packet
            case GET_PACKET_DATA:
                if (dataLength < PACKET_LENGTH)
                {
                    // Wait for receiving a byte
                    if(RCIF == 1)
                    {
                        data[dataLength] = RCREG; // there is bug when receiving more than 2 bytes, UART hangs
                        dataLength ++;
                    }
                    // if time out occurs ...
                    // if complete packet is not received within 500ms
                    else if (time > 50)
                    {
                        state = IDLE;
                        #ifdef SERIAL_DEBUG
                        serial_Putstr("TIMEOUT\n",8);
                        #endif
                    }
                }
                // If all bytes are received ...
                else
                {
                    // read address from DIP switch
                    address = BOARD_ID;
                    // Go to the next state
                    state = CHECK_CRC;
                    #ifdef SERIAL_DEBUG
                    serial_Putstr("CRCCHK\n",7);
                    #endif
                }
                break;
                
            case CHECK_CRC:
                // If CRC is OK
                if (CheckCRC(data,dataLength) == OK)
                {
                    // Check Address ...
                    if ((data[CMD_POS_ADDR] == address)||(data[CMD_POS_ADDR] == GLOBAL_ADDRESS))
                    {
                        CmdId = data[CMD_POS_CMD];

                        state = EXECUTE_COMMAND;
                        #ifdef SERIAL_DEBUG
                        serial_Putstr("EXECMD\n",7);
                        #endif
                    }
                    else
                    {
                        state = IDLE;
                        #ifdef SERIAL_DEBUG
                        serial_Putstr("ADDRFAIL\n",9);
                        #endif
                    }
                }
                else
                {
                    state = IDLE;
                    #ifdef SERIAL_DEBUG
                    serial_Putstr("CRCFAIL\n",8);
                    #endif
                }
                break;
                
            case EXECUTE_COMMAND:
                switch (CmdId)
                {
#ifndef NEW_PCB_BOARD
                    case CONTROL_LED:
                        if(data[CMD_POS_DATA1])
                        {
                            LED_STATUS = ENABLE_ACTIVE_HIGH;
                        }
                        else
                        {
                            LED_STATUS = DISABLE_ACTIVE_HIGH;
                        }
                        state = IDLE;
                        break;
#endif
                    case RESET_MAIN_MCU:
                        //house-keeping here
                        state = IDLE;
                        RESET();
                        break;
                    default:
                        state = IDLE;
                        break;
                }
                break;
                
            default:
                state = IDLE;
                break;
        }
    }
}
