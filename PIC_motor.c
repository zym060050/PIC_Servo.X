#include "PIC_Servo.h"



void PIC_Motor_Control(unsigned char target_A_B, unsigned char control, unsigned long position)
{
    (void)position;
    unsigned char M1 = 0;
    unsigned char M2 = 0;
    motorACurrentPos = MotorA_Position;
    motorBCurrentPos = MotorB_Position;
    switch(control)
    {
        case MOTOR_CONTROL_FW:
            M1 = ENABLE_ACTIVE_LOW;
            M2 = DISABLE_ACTIVE_LOW;
            break;
        case MOTOR_CONTROL_BW:
            M1 = DISABLE_ACTIVE_LOW;
            M2 = ENABLE_ACTIVE_LOW;
            position=position*-1;
            break;
        case MOTOR_CONTROL_STOP:
            M1 = DISABLE_ACTIVE_LOW;
            M2 = DISABLE_ACTIVE_LOW;
            break;
        default:
            M1 = DISABLE_ACTIVE_LOW;
            M2 = DISABLE_ACTIVE_LOW;
            break;
    }
    
    if(target_A_B == MOTOR_A)
    {
        motorATargetPos = motorACurrentPos + position;
        //float stop first
        M_A1 = ENABLE_ACTIVE_LOW;
        M_A2 = ENABLE_ACTIVE_LOW;
        //take action
        M_A1 = M1;
        M_A2 = M2;
    }
    else if(target_A_B == MOTOR_B)
    {
        motorBTargetPos = motorBCurrentPos + position;
        //float stop first
        M_B1 = ENABLE_ACTIVE_LOW;
        M_B2 = ENABLE_ACTIVE_LOW;
        //take action
        M_B1 = M1;
        M_B2 = M2;
    }
}
