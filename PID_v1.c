
#include "PID_v1.h"
#include <stdbool.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void PID_PID(PID *target, double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
{
	
    target->myOutput = Output;
    target->myInput = Input;
    target->mySetpoint = Setpoint;
	target->inAuto = 0;
	
	PID_SetOutputLimits(target, -80, 80);				//default output limit corresponds to 
												//the arduino pwm limits

    target->SampleTime = 100;							//default Controller Sample Time is 0.1 seconds
    
    PID_SetControllerDirection(target, ControllerDirection);
    PID_SetTunings(target, Kp, Ki, Kd);
    PID_SetSampleTime(target, 10);

    //lastTime = millis()-SampleTime;				
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
void PID_Compute(PID *target)
{
   //if(!inAuto) return 0;
   //unsigned long now = millis();
   //unsigned long timeChange = (now - lastTime);
   //if(timeChange>=SampleTime)
   //{
      /*Compute all the working error variables*/
	  double input = *(target->myInput);
      double error = *(target->mySetpoint) - input;
      target->ITerm+= (target->ki * error);
      if(target->ITerm >target->outMax) target->ITerm= target->outMax;
      else if(target->ITerm < target->outMin) target->ITerm= target->outMin;
      double dInput = (input - target->lastInput);
 
      /*Compute PID Output*/
      double output = target->kp * error + target->ITerm- target->kd * dInput;
      
	  if(output > target->outMax) output = target->outMax;
      else if(output < target->outMin) output = target->outMin;
	  *(target->myOutput) = output;
	  
      /*Remember some variables for next time*/
      target->lastInput = input;
      //lastTime = now;
	  //return 1;
   //}
   //else return false;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID_SetTunings(PID *target, double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   target->dispKp = Kp; target->dispKi = Ki; target->dispKd = Kd;
   
   double SampleTimeInSec = ((double)target->SampleTime)/1000;  
   target->kp = Kp;
   target->ki = Ki * SampleTimeInSec;
   target->kd = Kd / SampleTimeInSec;
 
  if(target->controllerDirection ==REVERSE)
   {
      target->kp = (0 - target->kp);
      target->ki = (0 - target->ki);
      target->kd = (0 - target->kd);
   }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID_SetSampleTime(PID *target, int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)target->SampleTime;
      target->ki *= ratio;
      target->kd /= ratio;
      target->SampleTime = (unsigned long)NewSampleTime;
   }
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_SetOutputLimits(PID *target, double Min, double Max)
{
   if(Min >= Max) return;
   target->outMin = Min;
   target->outMax = Max;
 
   if(target->inAuto)
   {
	   if(*(target->myOutput) > target->outMax) *(target->myOutput) = target->outMax;
	   else if(*(target->myOutput) < target->outMin) *(target->myOutput) = target->outMin;
	 
	   if(target->ITerm > target->outMax) target->ITerm= target->outMax;
	   else if(target->ITerm < target->outMin) target->ITerm= target->outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID_SetMode(PID *target, int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !target->inAuto)
    {  /*we just went from manual to auto*/
        PID_Initialize(target);
    }
    target->inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID_Initialize(PID *target)
{
   target->ITerm = *(target->myOutput);
   target->lastInput = *(target->myInput);
   if(target->ITerm > target->outMax) target->ITerm = target->outMax;
   else if(target->ITerm < target->outMin) target->ITerm = target->outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID_SetControllerDirection(PID *target, int Direction)
{
   if(target->inAuto && Direction !=target->controllerDirection)
   {
	  target->kp = (0 - target->kp);
      target->ki = (0 - target->ki);
      target->kd = (0 - target->kd);
   }   
   target->controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID_GetKp(PID *target){ return  target->dispKp; }
double PID_GetKi(PID *target){ return  target->dispKi;}
double PID_GetKd(PID *target){ return  target->dispKd;}
int PID_GetMode(PID *target){ return  target->inAuto ? AUTOMATIC : MANUAL;}
int PID_GetDirection(PID *target){ return target->controllerDirection;}


