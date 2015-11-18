// PIC18F2520 Configuration Bit Settings

// 'C' source line config statements
#include <xc.h>
//device is selected in project property
//#include <PIC18F2520.h> 

#define xBREAD_BOARD_SETUP
#define xNEW_PCB_BOARD
/*Testing Feature Switch*/
#define xSERIAL_ECHO_TEST
#define xSERIAL_DEBUG

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1H
#ifdef BREAD_BOARD_SETUP
#pragma config OSC = HS
#else
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#endif
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#ifdef BREAD_BOARD_SETUP
#pragma config MCLRE = ON       // MCLR Pin Enable bit (RE3 input pin disabled; MCLR enabled)
#else
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)
#endif

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)



#ifdef BREAD_BOARD_SETUP
#define _XTAL_FREQ 20000000
#else
#define _XTAL_FREQ 40000000
#endif



// Defining IOs
#ifndef NEW_PCB_BOARD
#define PWMA         RC2
#define PWMB         RC1
#define M_A1         RB7
#define M_A2         RB6
#define M_B1         RB3
#define M_B2         RB2

#define MA_Tacho0    RA1
#define MA_Tacho1    RA2
#define MA_TachoXOR  RA4
#define MB_Tacho0    RA3
#define MB_Tacho1    RA5
#define MB_TachoXOR  RC0

#define BOARD_ID     RE3
#define RS485_DE     RB5 //RS485 write_enable active high, DE and RE shorted.

#define LED_STATUS   RA0
#else
//new board
#define PWMA         RC2
#define PWMB         RC1
#define M_A1         RB7
#define M_A2         RB6
#define M_B1         RB5
#define M_B2         RB4

#define MA_Tacho0    RA1
#define MA_Tacho1    RA2
#define MA_TachoXOR  RA4
#define MB_Tacho0    RA3
#define MB_Tacho1    RA5
#define MB_TachoXOR  RC0

#define BOARD_ID     RB3
#define RS485_DE     RC5 //RS485 write_enable active high, DE and RE shorted.


#define SPARE_IN     RE3
#define EXT_INTIO    RB2
#define EXT_INT1     RB1
#define EXT_INT0     RB0
#define VMOT_MON     RA0
#endif


// Be careful about baudrate error. Depending on the Fosc some baudrates can not be generated
// Below you can find a list of avialable baudrates for 16MHz and 20MHz:
// 16MHz: 1200, 2400, 4800, 9600, 19200, 250000
// 20MHz:     , 2400, 4800,
// Note: Only Low baudrate mode (BRGH = 0) is considered.
// Rev2 set BRGH=1 & use baud 19200
#define RS485_BAUDRATE          19200
#define PACKET_LENGTH           7
#define GLOBAL_ADDRESS          0xFF
#define CRC_POLYNOMIAL          0x1021 // CRC-CCITT, can change to CRC-16
#define CRC_INITIALE_VALUE      0x0000

#define ENABLE_ACTIVE_LOW       0
#define ENABLE_ACTIVE_HIGH      1
#define DISABLE_ACTIVE_LOW      1
#define DISABLE_ACTIVE_HIGH     0


/*T3 to give 10ms every interrupt*/
#define T3_START_COUNT_LO 0x2C
#define T3_START_COUNT_HI 0xCF


//CMD Structure
/*
 * SOH      BORAD_ADDR      CMD     DATA1       DATA2       CRC
 */
#define CMD_POS_SOH         0
#define CMD_POS_ADDR        1
#define CMD_POS_CMD         2
#define CMD_POS_DATA1       3
#define CMD_POS_DATA2       4
#define CMD_POS_CRC         5
/**/
#define OK      0
#define ERROR   0xFF
#define SOH     0x55
/*CMD ID*/
#ifndef NEW_PCB_BOARD
#define CONTROL_LED                 0x1E
#endif
#define RESET_MAIN_MCU              0x1F
/*
 * Command example
//LED OFF
0x55 0xFF 0x1E 0x00 0x00 0x25 0xA4
0x55 0x01 0x1E 0x00 0x00 0x18 0xB3

//LED ON
0x55 0xFF 0x1E 0x01 0x00 0x16 0x95
0x55 0x01 0x1E 0x01 0x00 0x2B 0x82

//RESET
0x55 0xFF 0x1F 0x00 0x00 0x12 0x94
0x55 0x01 0x1F 0x00 0x00 0x2F 0x83
 */


//extern
extern unsigned char state;
// Defining program state constants
#define IDLE            0
#define GET_PACKET_DATA 1
#define CHECK_CRC       2
#define EXECUTE_COMMAND 3
#define RESEND_COMMAND  4
#define SHOW_ERROR      5



//Functions
void Initialize(void);
unsigned char CheckCRC(unsigned char *inputData, unsigned char inputDataLength);
unsigned int GenerateCRC (unsigned char *inputData, unsigned char inputDataLength);
void serial_Putch(unsigned char byte);
void serial_Putstr(const char *str, unsigned char length);
