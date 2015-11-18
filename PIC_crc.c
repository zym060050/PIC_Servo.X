#include "PIC_Servo.h"

//For calculating CRC for a string, append 16 zero bit to the end of string
//For checking the CRC of a string, pass all string including CRC to this routine. If the output is 0, the CRC is OK
static unsigned int GenerateCRC (unsigned char *inputData,unsigned char inputDataLength)
{
    unsigned int remainder,msBit;
    unsigned char i,j,mask;

    remainder = CRC_INITIALE_VALUE;
    for (i=0;i<inputDataLength;i++)
    {
        mask = 0x80;
        // Do for all bits in each byte
        for (j=0;j<8;j++)
        {
            msBit = remainder & 0x8000;
            remainder = remainder << 1;
            if ((inputData[i] & mask) != 0)
                remainder = remainder | 0x0001;
            mask = mask >> 1;
            if (msBit != 0)
                remainder = remainder ^ CRC_POLYNOMIAL;
        }
    }
    return(remainder);
}

unsigned char CheckCRC(unsigned char *inputData,unsigned char inputDataLength)
{
    if (GenerateCRC(inputData,inputDataLength) == 0)
    {
        return(OK);
    }
    else
    {
        return(ERROR);
    }
}
