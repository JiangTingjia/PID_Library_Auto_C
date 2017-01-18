/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Auto.h
*Note: This file is used for PID Autoning
----------------------------------------------------------------------------------------*/
#ifndef PID_Auto_H
#define PID_Auto_H

#include "stdbool.h"

typedef struct
{
	/***********************************variables****************************/
	double noiseBand;			// only the |SP - PV| upper this threhold, the relay output change 
	int controlType;			// 1 : PID  0: PI
	int nLookBack;		// must > 3 
	double oStep;		// PWM for Output step, Output = mean + step  or Output = mean - step
	double outputStart;		//  mean value for Output
	//
	unsigned int *Current_time;		// current system time
	double *input, *output, *setpoint;		// PV  CV SP
	bool isMax, isMin;	 // mark for judge peak or not
	bool running;		// mark for auto process is running			
	unsigned int peak1, peak2, lastTimeAuto;	// peak Value (upper and lower)		
	unsigned int sampleTime;		// Calculate periods
	int peakType;		// upper or lower mark
	double lastInputs[101];		
	double peaks[10];		
	int peakCount;		// peaks count
	bool justchanged;	
	bool justevaled;	// mark for auto end
	double absMax, absMin;	// peak	Value absolute
	double Ku, Pu;		
}TEMP_AUTO_STRUCT;

/**********************************functions***************************/
/*------------------------------------------------------------------------------
*Function: PID_ATune
* Brief: Initial PID AutoTuner
* Param: Input  IN  PV
*        Output  IN  CV
* Retval:  none
----------------------------------------------------------------------------------*/
void PID_ATune(double* Input, double* Output, double *setpoint, unsigned int *iCurTime, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: Runtime
* Brief: Process for PID Autotuner
* Param: sTempAuto	IN  A Autotuner instance
* Retval:  0£ºProcess end
*		   1£ºProcessing
----------------------------------------------------------------------------------*/
int Runtime(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: Cancel
* Brief: Cancel Autotuner
* Param: sTempAuto	IN A Autotuner instance
* Retval:  none
----------------------------------------------------------------------------------*/
void Cancel(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetOutputStep
* Brief: Set step
* Param: Step  IN  
*        sTempAuto	IN A Autotuner instance
* Retval:  none
----------------------------------------------------------------------------------*/
void SetOutputStep(double Step, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetOutputStep
* Brief: Get Step
* Param:   sTempAuto	IN A Autotuner instance
* Retval:  step
----------------------------------------------------------------------------------*/
double GetOutputStep(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetStartOutput
* Brief: Set mean Value
* Param: StartVal  IN  mean Value
*        sTempAuto	IN Autotuner instance
* Retval:  none
----------------------------------------------------------------------------------*/
void SetStartOutput(double StartVal, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GettStartOutput
* Brief: Get mean Value
* Param:  sTempAuto	IN Autotuner instance
* Retval:  mean Value
----------------------------------------------------------------------------------*/
double GettStartOutput(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetControlType
* Brief: Set PID or PI Control
* Param: Type  IN  1:PID,  0:PI
*        sTempAuto	IN Autotuner instance
* Retval:  none
----------------------------------------------------------------------------------*/
void SetControlType(int Type, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetControlType
* Brief: Get ControlType
* Param: sTempAuto	IN Autotuner instance
* Retval:  0£ºPI
*		   1£ºPID
----------------------------------------------------------------------------------*/
int GetControlType(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetLookbackSec
* Brief: Set lookbackValue
* Param: value  IN  
*        sTempAuto	IN Autotuner instance
* Retval:  none
----------------------------------------------------------------------------------*/
void SetLookbackSec(int value, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetLookbackSec
* Brief: Get lookbackValue
* Param: sTempAuto	IN Autotuner instance
* Retval:  lookbackValue
----------------------------------------------------------------------------------*/
int GetLookbackSec(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetNoiseBand
* Brief: SetNoiseBand
* Param: Band  IN  NoiseBand
*        sTempAuto	IN Autotuner instance
* Retval:  none
----------------------------------------------------------------------------------*/
void SetNoiseBand(double Band, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetNoiseBand
* Brief: GetNoiseBand
* Param: sTempAuto	IN Autotuner instance
* Retval:  NoiseBand
----------------------------------------------------------------------------------*/
double GetNoiseBand(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetKpAuto/GetKiAuto/GetKdAuto
* Brief: Get Kp Ki Kd
* Param: sTempAuto	IN Autotuner instance
* Retval:  Kp/kI/Kd
----------------------------------------------------------------------------------*/
double GetKpAuto(TEMP_AUTO_STRUCT *sTempAuto);										
double GetKiAuto(TEMP_AUTO_STRUCT *sTempAuto);										
double GetKdAuto(TEMP_AUTO_STRUCT *sTempAuto);										

/*------------------------------------------------------------------------------
*Function: FinishUp
* Brief: Calculate Ku Pu
* Param: sTempAuto	IN Autotuner instance
* Retval:  none
----------------------------------------------------------------------------------*/
void FinishUp(TEMP_AUTO_STRUCT *sTempAuto);


#endif

