#include "PIC_Servo.h"

// Initialize Function
void Initialize(void)
{
    int i=0;

	// Initialize GPIO
#ifndef NEW_PCB_BOARD
    PORTA = 0x1F;
    TRISA = 0xFE;
    
    PORTB = 0xDF;
    TRISB = 0x13;
    
    PORTC = 0;
    TRISC = 0xF9;//0xB9;
    
    //set all analog pins as digital 
    ADCON1 = 0x0F;
    
    // status LED on
    LED_STATUS = LED_ON;
#else
    //new board
    PORTA = 0x1F;
    TRISA = 0xFF;
    
    PORTB = 0xF3;
    TRISB = 0x0F;
    
    PORTC = 0;
    TRISC = 0x81;
    
    //set all analog pins as digital except for AN0  
    ADCON1 = 0x0E;
#endif
    
    
    // RS485 read standby
    RS485_DE = DISABLE;
    
    
	// Initialize Serial Port
    // Asynchronous mode (bit 4), 8-bit transmission (bit 6), Transmit disabled (bit 5), High speed baud rate (bit 2)
    TXSTA = 0x04;
    //TXSTA = 0b00100100;
    // Serial port enabled (bit 7), 8-bit reception (bit 6), continuous receive disabled (bit 4)
    RCSTA = 0x80;
    //RCSTA = 0b10010000;
    /*For BRG16 == 0*/
    // Baud rate = Fosc/(16*(SPBRG+1))
    //SPBRG = ((_XTAL_FREQ/16)/RS485_BAUDRATE)-1;
    /*For BRG16 == 1*/
    BRG16 = SET;
#ifdef BREAD_BOARD_SETUP
    /*19200: 259*/
    SPBRGH = 0x01;
    SPBRG = 0x03;
    /*9600: 520*/
    //SPBRGH = 0x02;
    //SPBRG = 0x08;
#else
    //SPBRGH:SPBRG = ((_XTAL_FREQ/4)/RS485_BAUDRATE)-1;
    /*19200: 520*/
    SPBRGH = 0x02;
    SPBRG = 0x08;
    /*9600: 1040*/
    //SPBRGH = 0x04;
    //SPBRG = 0x10;
#endif

	// Initialize I2C (TBD)
    
    
	// Initialize Timer2 => PWM timer
    // post-scaler = 1, pre-scaler = 16, TMR2ON =0
    T2CON = 0x02;
    // Set period register for 128us, (128*(_XTAL_FREQ/4)/16)/1000000)-1;
    PR2 = 0x4F;
    //Clear timer 2 register
    TMR2 = CLEAR;
      
    //initialize PWM module
    CCP1CON = 0x0C;
    CCP2CON = 0x0C;
    
    //set below for PWM duty cycles 0~80 corresponds to 0~100% duty.
    CCPR1L = 0x1E; //30/80
    CCPR2L = 0x1E; //40/80       
    
    
    //timer 3 interrupt    
    T3CON = 0xF9;
    //Start count 64286 -> 10ms
    TMR3L = T3_START_COUNT_LO;
    TMR3H = T3_START_COUNT_HI;        
    TMR3IE = ENABLE;           
    IPEN = ENABLE;
    TMR3IP = 0;  //must set as low priority interrupt.         
    TMR3IF = CLEAR;
    
        
	// Start System ...
         
    // Initialize State Variables
    state = IDLE;
    //Initialize motor controls
    M_A1 = ENABLE_ACTIVE_LOW;
    M_A2 = DISABLE_ACTIVE_LOW;
    M_B1 = ENABLE_ACTIVE_LOW;
    M_B2 = DISABLE_ACTIVE_LOW;     
    
    
    // Start timer 2 (PWM)
    TMR2ON = ENABLE;
    
    
#ifndef NEW_PCB_BOARD
    // Turn off status LED => To show end of initialization
    for(i=0;i<10;i++)
    {
        __delay_ms(19);
    }
    LED_STATUS = LED_OFF;
#endif
    
    
    // Enable serial port receiver
    TXEN = ENABLE;
    CREN = ENABLE;
    RCIE = DISABLE;
    // Enable pripheral Interrupts
    PEIE = ENABLE;
    //Enable all longerrupts
    GIE = ENABLE;
    
    
    // clear WDT
    //CLRWDT(); 
}
