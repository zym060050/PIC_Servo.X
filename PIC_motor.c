#include "PIC_Servo.h"

#ifdef ENABLE_PID_CONTROL
#include "PID_v1.h"

double MOTOR_A_PID_OUTPUT = 0;
double motorA_setpoint = 0;
double motorA_pos = 0;
#endif

long MotorA_Position = 0;
long motorATargetPos = 0;
long MotorB_Position = 0;
long motorBTargetPos = 0;

void PIC_Motor_Control(unsigned char target_A_B, unsigned char control, unsigned long move_steps)
{
    unsigned char M1 = 0;
    unsigned char M2 = 0;
    
    //if no steps move when the control is not STOP, do nothing
    if((move_steps == 0) && (control != MOTOR_CONTROL_STOP))
        return;
    
    switch(control)
    {
        case MOTOR_CONTROL_FW:
            M1 = ENABLE_ACTIVE_LOW;
            M2 = DISABLE_ACTIVE_LOW;
            if(target_A_B == MOTOR_A)
            {
                motorATargetPos = MotorA_Position + move_steps;
            }
            else if(target_A_B == MOTOR_B)
            {
                motorBTargetPos = MotorB_Position + move_steps;
            }
            break;
        case MOTOR_CONTROL_BW:
            M1 = DISABLE_ACTIVE_LOW;
            M2 = ENABLE_ACTIVE_LOW;
            if(target_A_B == MOTOR_A)
            {
                motorATargetPos = MotorA_Position - move_steps;
            }
            else if(target_A_B == MOTOR_B)
            {
                motorBTargetPos = MotorB_Position - move_steps;
            }
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
#ifndef ENABLE_PID_CONTROL
        PIC_Motor_Speed_Configure(MOTOR_A, move_steps);
#endif
        //float stop first
        M_A1 = ENABLE_ACTIVE_LOW;
        M_A2 = ENABLE_ACTIVE_LOW;
        //take action
        M_A1 = M1;
        M_A2 = M2;
    }
    else if(target_A_B == MOTOR_B)
    {
#ifndef ENABLE_PID_CONTROL
        PIC_Motor_Speed_Configure(MOTOR_B, move_steps);
#endif
        //float stop first
        M_B1 = ENABLE_ACTIVE_LOW;
        M_B2 = ENABLE_ACTIVE_LOW;
        //take action
        M_B1 = M1;
        M_B2 = M2;
    }
}

void PIC_Motor_Move_To_Position(unsigned char target_A_B, long position)
{
    if(target_A_B == MOTOR_A)
    {
        if(position>MotorA_Position)
        {
            PIC_Motor_Control(target_A_B, MOTOR_CONTROL_FW, (position-MotorA_Position));
        }
        else if (position < MotorA_Position)
        {
            PIC_Motor_Control(target_A_B, MOTOR_CONTROL_BW, (MotorA_Position-position));
        }
    }
    else if(target_A_B == MOTOR_B)
    {
        if(position>MotorB_Position)
        {
            PIC_Motor_Control(target_A_B, MOTOR_CONTROL_FW, (position-MotorB_Position));
        }
        else if (position < MotorB_Position)
        {
            PIC_Motor_Control(target_A_B, MOTOR_CONTROL_BW, (MotorB_Position-position));
        }
    }
}

void PIC_Motor_Speed_Configure(unsigned char target_A_B, unsigned long steps_delta)
{
    if(target_A_B == MOTOR_A)
    {
        //decide speed
        if(steps_delta >= 180)
        {
            if (CCPR1L != MOTOR_MAX_SPEED)
                CCPR1L = MOTOR_MAX_SPEED;
        }
        else if (steps_delta >= 20)
        {
            if (CCPR1L != MOTOR_SLOW_SPEED)
                CCPR1L = MOTOR_SLOW_SPEED;
        }
        else if (steps_delta >= 10)
        {
            if (CCPR1L != MOTOR_VERY_SLOW_SPEED)
                CCPR1L = MOTOR_VERY_SLOW_SPEED;
        }
        else if (steps_delta < 2)
        {
            if (CCPR1L != MOTOR_SUPER_SLOW_SPEED)
                CCPR1L = MOTOR_SUPER_SLOW_SPEED;
        }
    }
    else if(target_A_B == MOTOR_B)
    {
        //decide speed
        if(steps_delta >= 180)
        {
            if (CCPR2L != MOTOR_MAX_SPEED)
                CCPR2L = MOTOR_MAX_SPEED;
        }
        else if (steps_delta >= 20)
        {
            if (CCPR2L != MOTOR_SLOW_SPEED)
                CCPR2L = MOTOR_SLOW_SPEED;
        }
        else if (steps_delta >= 10)
        {
            if (CCPR2L != MOTOR_VERY_SLOW_SPEED)
                CCPR2L = MOTOR_VERY_SLOW_SPEED;
        }
        else if (steps_delta < 2)
        {
            if (CCPR2L != MOTOR_SUPER_SLOW_SPEED)
                CCPR2L = MOTOR_SUPER_SLOW_SPEED;
        }
    }
}

#ifdef ENABLE_PID_CONTROL
void PIC_Motor_PID_Loop()
{
    /*Motor A PID*/
    if(MotorA_Position==motorATargetPos)
    {
        PIC_Motor_STOP(MOTOR_A);
    }
    else
    {
        motorA_pos=MotorA_Position;
        motorA_setpoint=motorATargetPos;
        PID(&motorA_pos, &MOTOR_A_PID_OUTPUT, &motorA_setpoint,MOTOR_PID_KP,MOTOR_PID_KI, MOTOR_PID_KD, DIRECT);
        PID_Compute();
        if(MOTOR_A_PID_OUTPUT==0)
        {
            PIC_Motor_STOP(MOTOR_A);
        }
        else if(MOTOR_A_PID_OUTPUT>0)
        {
            PIC_Motor_FW(MOTOR_A);
        }
        else
        {
            PIC_Motor_BW(MOTOR_A);
            MOTOR_A_PID_OUTPUT = MOTOR_A_PID_OUTPUT*(-1);
        }
        CCPR1L=(unsigned char)(MOTOR_A_PID_OUTPUT);
    }
    /*Motor B PID*/
    //TODO:
}

void PIC_Motor_FW(unsigned char target_A_B)
{
    if(target_A_B == MOTOR_A)
    {
        M_A1 = ENABLE_ACTIVE_LOW;
        M_A2 = DISABLE_ACTIVE_LOW;
    }
    else if(target_A_B == MOTOR_B)
    {
        M_B1 = ENABLE_ACTIVE_LOW;
        M_B2 = DISABLE_ACTIVE_LOW;
    }
}

void PIC_Motor_BW(unsigned char target_A_B)
{
    if(target_A_B == MOTOR_A)
    {
        M_A1 = DISABLE_ACTIVE_LOW;
        M_A2 = ENABLE_ACTIVE_LOW;
    }
    else if(target_A_B == MOTOR_B)
    {
        M_B1 = DISABLE_ACTIVE_LOW;
        M_B2 = ENABLE_ACTIVE_LOW;
    }
}

void PIC_Motor_STOP(unsigned char target_A_B)
{
    if(target_A_B == MOTOR_A)
    {
        M_A1 = DISABLE_ACTIVE_LOW;
        M_A2 = DISABLE_ACTIVE_LOW;
    }
    else if(target_A_B == MOTOR_B)
    {
        M_B1 = DISABLE_ACTIVE_LOW;
        M_B2 = DISABLE_ACTIVE_LOW;
    }
}
#endif
