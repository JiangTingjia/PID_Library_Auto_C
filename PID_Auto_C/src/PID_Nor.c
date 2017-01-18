/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Nor.h
*Note: This file is used for PID Autoning
----------------------------------------------------------------------------------------*/
#include "PID_Nor.h"

/*Constructor (...)*********************************************************
*    The parameters specified here are those for for which we can't set up
*    reliable defaults, so we need to have the user set them.
***************************************************************************/
void PID(double* Input, double* Output, double* Setpoint, unsigned int *iCurTime, TEMP_NOR_STRUCT *sTempNor)
{
	//default output limit corresponds to 
	//the arduino pwm limits
	double outMin = 0;
	double outMax = 255;	
	unsigned int SampleTime = 100;	//default Controller Sample Time is 0.1 seconds
	int ControllerDirection = DIRECT;
	double Kp = outMax * 0.01;
	double Ki = 0;
	double Kd = 0;
	int Mode = AUTOMATIC;
	sTempNor->Current_time = iCurTime;
	sTempNor->myOutput = Output;	
	sTempNor->myInput = Input;	
	sTempNor->mySetpoint = Setpoint;	
	sTempNor->inAuto = false;		
	SetOutputLimits(outMin, outMax, sTempNor);				
	SetSampleTime(SampleTime, sTempNor);
	SetControllerDirection(ControllerDirection, sTempNor);
	SetTunings(Kp, Ki, Kd, sTempNor);			
	SetMode(Mode, sTempNor);		
	sTempNor->lastTimeNor = *(sTempNor->Current_time) - sTempNor->SampleTime; 	
}

/* Compute() **********************************************************************
*     This, as they say, is where the magic happens.  this function should be called
*   every time "void loop()" executes.  the function will decide for itself whether a new
*   pid Output needs to be computed.  returns true when the output is computed,
*   false when nothing has been done.
**********************************************************************************/
bool Compute(TEMP_NOR_STRUCT *sTempNor)
{
	unsigned int now = 0;		
	unsigned int timeChange = 0;	
	
	if (!sTempNor->inAuto) 	
	{
		return false;	
	}		
	
	now = *(sTempNor->Current_time);		
	timeChange = (now - sTempNor->lastTimeNor);
	
	if (timeChange >= sTempNor->SampleTime)		
	{
		/*Compute all the working error variables*/
		double dInput = 0.0;
		double output = 0.0;
		double input = *(sTempNor->myInput);		
		double error = *(sTempNor->mySetpoint) - input;	
		sTempNor->ITerm += (sTempNor->kiNor * error);		
		if (sTempNor->ITerm > sTempNor->outMax) sTempNor->ITerm = sTempNor->outMax;
		else if (sTempNor->ITerm < sTempNor->outMin) sTempNor->ITerm = sTempNor->outMin;
		dInput = (input - sTempNor->lastInput);	
		/*Compute PID Output*/
		output = sTempNor->kpNor * error + sTempNor->ITerm - sTempNor->kdNor * dInput;	
		if (output > sTempNor->outMax) output = sTempNor->outMax;
		else if (output < sTempNor->outMin) output = sTempNor->outMin;
		*(sTempNor->myOutput) = output;		
		/*Remember some variables for next time*/
		sTempNor->lastInput = input;
		sTempNor->lastTimeNor = now;		
		return true;
	}
	else 
	{
		return false;
	}
}

/* SetTunings(...)*************************************************************
* This function allows the controller's dynamic performance to be adjusted.
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
******************************************************************************/
void SetTunings(double Kp, double Ki, double Kd, TEMP_NOR_STRUCT *sTempNor)
{
	double SampleTimeInSec = 0.0;			
	
	if (Kp < 0 || Ki < 0 || Kd < 0)  
	{
		return;
	}
	
	SampleTimeInSec = ((double)sTempNor->SampleTime) / 1000;
	sTempNor->kpNor = Kp;
	sTempNor->kiNor = Ki * SampleTimeInSec;
	sTempNor->kdNor = Kd / SampleTimeInSec;

	if (sTempNor->controllerDirection == REVERSE)
	{
		sTempNor->kpNor = (0 - sTempNor->kpNor);
		sTempNor->kiNor = (0 - sTempNor->kiNor);
		sTempNor->kdNor = (0 - sTempNor->kdNor);
	}
}

/* SetSampleTime(...) *********************************************************
* sets the period, in Milliseconds, at which the calculation is performed
******************************************************************************/
void SetSampleTime(unsigned int NewSampleTime, TEMP_NOR_STRUCT *sTempNor)
{
	// 改变采样时间
	if (NewSampleTime > 0)
	{
		double ratio = (double)NewSampleTime
			/ (double)sTempNor->SampleTime;
		sTempNor->kiNor *= ratio;
		sTempNor->kdNor /= ratio;
		sTempNor->SampleTime = (unsigned int)NewSampleTime;
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
void SetOutputLimits(double Min, double Max, TEMP_NOR_STRUCT *sTempNor)
{
	if (Min >= Max) 
	{
		return;
	}
	sTempNor->outMin = Min;
	sTempNor->outMax = Max;

	if (sTempNor->inAuto)
	{
		if (*(sTempNor->myOutput) > sTempNor->outMax)
		{
			*(sTempNor->myOutput) = sTempNor->outMax;
		}
		else if (*(sTempNor->myOutput) < sTempNor->outMin)
		{
			*(sTempNor->myOutput) = sTempNor->outMin;
		}

		if (sTempNor->ITerm > sTempNor->outMax)
		{
			sTempNor->ITerm = sTempNor->outMax;
		}
		else if (sTempNor->ITerm < sTempNor->outMin)
		{
			sTempNor->ITerm = sTempNor->outMin;
		}
	}
}

/* SetMode(...)****************************************************************
* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
* when the transition from manual to auto occurs, the controller is
* automatically initialized
******************************************************************************/
void SetMode(int Mode, TEMP_NOR_STRUCT *sTempNor)
{
	bool newAuto = (Mode == AUTOMATIC);
	
	if (newAuto == !sTempNor->inAuto)	
	{  
		Initialize(sTempNor);
	}
	sTempNor->inAuto = newAuto;
}

/* Initialize()****************************************************************
*	does all the things that need to happen to ensure a bumpless transfer
*  from manual to automatic mode.
******************************************************************************/
void Initialize(TEMP_NOR_STRUCT *sTempNor)
{
	TEMP_NOR_STRUCT sTmp = *sTempNor;
	sTempNor->ITerm = *(sTempNor->myOutput);
	sTempNor->lastInput = *(sTempNor->myInput);
	if (sTempNor->ITerm > sTempNor->outMax)
	{
		sTempNor->ITerm = sTempNor->outMax;
	}
	else if (sTempNor->ITerm < sTempNor->outMin)
	{
		sTempNor->ITerm = sTempNor->outMin;
	}
	*sTempNor = sTmp;
}

/* SetControllerDirection(...)*************************************************
* The PID will either be connected to a DIRECT acting process (+Output leads
* to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
* know which one, because otherwise we may increase the output when we should
* be decreasing.  This is called from the constructor.
******************************************************************************/
void SetControllerDirection(bool Direction, TEMP_NOR_STRUCT *sTempNor)
{
	TEMP_NOR_STRUCT sTmp = *sTempNor;
	if (sTempNor->inAuto && Direction != sTempNor->controllerDirection)
	{
		sTempNor->kpNor = (0 - sTempNor->kpNor);
		sTempNor->kiNor = (0 - sTempNor->kiNor);
		sTempNor->kdNor = (0 - sTempNor->kdNor);
	}
	sTempNor->controllerDirection = Direction;
	*sTempNor = sTmp;
}

/* Status Funcions*************************************************************
* Just because you set the Kp=-1 doesn't mean it actually happened.  these
* functions query the internal state of the PID.  they're here for display
* purposes.  this are the functions the PID Front-end uses for example
******************************************************************************/
double GetKpNor(TEMP_NOR_STRUCT *sTempNor)
{ 
	return  sTempNor->dispKp;
}
double GetKiNor(TEMP_NOR_STRUCT *sTempNor)
{ 
	return  sTempNor->dispKi;
}
double GetKdNor(TEMP_NOR_STRUCT *sTempNor)
{ 
	return  sTempNor->dispKd;
}
int GetMode(TEMP_NOR_STRUCT *sTempNor)
{ 
	// 如果关闭，则关闭，如果其他，则开启
	return  sTempNor->inAuto ? AUTOMATIC : MANUAL;
}
int GetDirection(TEMP_NOR_STRUCT *sTempNor)
{ 
	return sTempNor->controllerDirection;
}

//**************ENDFILE****************//
