#include "PIC_Servo.h"

/*
 code to test TX
 * 
 *    	// Enable serial port transmitter
   		TXEN = ENABLE;
    	// enable RS485 transmitter
   		RS485_DE = ENABLE;
			while (!TXIF);
			TXREG = 0x6F;
			while (!TRMT);						
  		// Disable serial port transmitter
   		TXEN = DISABLE;
     	// disable RS485 transmitter
   		RS485_DE = DISABLE;

 * 
 */

#if 1
void serial_Putch(unsigned char byte)
{
    // Enable serial port transmitter
    //TXEN = ENABLE;
    // enable RS485 transmitter
    RS485_DE = ENABLE;
        while (!TXIF);
        TXREG = byte;
        while (!TRMT);						
    // Disable serial port transmitter
    //TXEN = DISABLE;
    // disable RS485 transmitter
    RS485_DE = DISABLE;
}

void serial_Putstr(const char *str, unsigned char length)
{
  RS485_DE = ENABLE;
  for(unsigned char i=0;i<length;i++)
  {
    //Wait for TXREG Buffer to become available
    while(!TXIF);

    //Write data
    TXREG=(*str);

    //Next goto char
    str++;
    
    while (!TRMT);
  }
  RS485_DE = DISABLE;
}

void serial_Putint() {
    int data = MotorA_Position;
    RS485_DE = ENABLE;
    TXREG = data & 0xff;
    while(TXIF==0); 
    TXREG = (data >> 8) & 0xff;
    while(TXIF==0);
        
    RS485_DE = DISABLE;
}
#else
void serial_Putch(unsigned char byte) 
{
  /* output one byte */
  while(!TXIF)	/* set when register is empty */
    continue;
  TXREG = byte;
}

void serial_Putstr(const char *str, unsigned char length)
{
  for(unsigned char i=0;i<length;i++)
  {
    //Wait for TXREG Buffer to become available
    while(!TXIF);

    //Write data
    TXREG=(*str);

    //Next goto char
    str++;
  }
}
#endif
