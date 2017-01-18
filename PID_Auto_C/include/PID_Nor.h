/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Nor.h
*Note: This file is used for PID Autoning
----------------------------------------------------------------------------------------*/
#ifndef PID_Nor_h
#define PID_Nor_h
#include "stdbool.h"

#define AUTOMATIC	1		//  Open PID Control
#define MANUAL	0			//  Close PID Control ,Output = last Output
#define DIRECT  0			//	Output and input have the same direction
#define REVERSE  1			//  Output and input have the reverse direction

///***********************************variables****************************/
typedef struct
{
	double kpNor;                  // Kp
	double kiNor;                  // Ki
	double kdNor;                  // Kd
	bool controllerDirection;		  // DIRECT  or REVERSE
	unsigned int SampleTime;	// unit: ms
	double outMin, outMax;		// Output Limit
	bool inAuto;					// AUTOMATIC  or MANUAL
	double *myInput;              //  SP
	double *myOutput;             //  PV
	double *mySetpoint;           //  CV 
	unsigned int *Current_time;		// unit :ms system time
	unsigned int lastTimeNor;	 //  last calculate time
	double ITerm;				// integration Value
	double lastInput;			// last PV 
	// 标志位
	double dispKp;				// Kp: Only used for display 
	double dispKi;				// Ki: Only used for display 
	double dispKd;				// Kd: Only used for display 
}TEMP_NOR_STRUCT;

/**********************************functions***************************/
/*------------------------------------------------------------------------------
*Function: PID
* Brief: Initial PID Control system
* Param: Input  IN  PV
*        Output  IN  CV
*        Setpoint  IN  SP
*        Kp  IN  P Param
*        Ki  IN  I Param
*        Kd  IN  D Param
*        ControllerDirection  IN  Direction or Reverse
* Retval:  none
----------------------------------------------------------------------------------*/
void PID(double* Input, double* Output, double* Setpoint, unsigned int *iCurTime, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: Initialize
* Brief: this function is called when sampletime is changed 
* Param: none
* Retval: none
----------------------------------------------------------------------------------*/
void Initialize(TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: Compute
* Brief: Calculate PID output
* Param: 无
* Retval:  sucess：true
*		   failed：false
----------------------------------------------------------------------------------*/
bool Compute(TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetMode
* Brief: change Control Mode AUTOMATIC or MANUAL
* Param: Mode  IN  AUTOMATIC or MANUAL
* Retval:  none
----------------------------------------------------------------------------------*/
void SetMode(int Mode, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: GetMode
* Brief: get AUTOMATIC or MANUAL
* Param: none
* Retval: 1: AUTOMATIC
*         0：MANUAL
----------------------------------------------------------------------------------*/
int GetMode(TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetOutputLimits
* Brief: Set OutputLimits default 0 -> 255
* Param: Min  IN  lower
*        Max  IN  upper
* Retval:  none
----------------------------------------------------------------------------------*/
void SetOutputLimits(double Min, double Max, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetSampleTime
* Brief: change sample time ， default:100ms
* Param: NewSampleTime  IN  unit(ms)
* Retval:  none
----------------------------------------------------------------------------------*/
void SetSampleTime(unsigned int NewSampleTime, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetTunings
* Brief: change Kp Ki kD
* Param: Kp  IN  P
*        Ki  IN  I
*        Kd  IN  D
* Retval:  none
----------------------------------------------------------------------------------*/
void SetTunings(double Kp, double Ki, double Kd, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetControllerDirection
* Brief: Set DIRECT  or REVERSE
* Param: Direction  IN  DIRECT  or REVERSE
* Retval:  none
----------------------------------------------------------------------------------*/
void SetControllerDirection(bool Direction, TEMP_NOR_STRUCT *sTempNor);

double GetKpNor(TEMP_NOR_STRUCT *sTempNor);						  // These functions query the pid for interal values.

double GetKiNor(TEMP_NOR_STRUCT *sTempNor);						  //  they were created mainly for the pid front-end,

double GetKdNor(TEMP_NOR_STRUCT *sTempNor);

int GetDirection(TEMP_NOR_STRUCT *sTempNor);

#endif

