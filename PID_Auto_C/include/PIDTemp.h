/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Nor.h
*Note: This file is used for PID Autoning
----------------------------------------------------------------------------------------*/
#include "PID_Nor.h"
#include "PID_Auto.h"

typedef struct
{
	double KP;			//Kp
	double KI;			//Ki
	double KD;			//Kd
	double dMin;		// lower bound
	double dMax;		// max bound
	unsigned int iSampleTimeNor;		// calculate periods
	int iDirection;						// DIRECT  or REVERSE
	int iMode;							// AUTOMATIC  or MANUAL
	double aTuneStep;					// PWM for Output step, Output = mean + step  or Output = mean - step
	double aTuneNoise;					// only the |SP - PV| upper this threhold, the relay output change 
	double aTuneStartValue;				// mean value for Output
	unsigned int aTuneLookBack;		// must > 3 
	int iType;						// PI: 0   PID: 1
	unsigned int iSinTimes;  	// unit:ms	if runningtime > iSinTimes, running auto tuner
	unsigned int iOKTimes;		// unit:ms  judge runningtime > iOKTimes or not, since autotuner complete
}PID_INIT_PARM;

typedef struct
{
	double *inputRun;		// PV		
	double *outputRun;		// CV
	double *setpointRun;	// SP
	TEMP_NOR_STRUCT sTempNor;	// PID control instance
	TEMP_AUTO_STRUCT sTempAuto;	// PID auto instance
	int ATuneModeRemember;	// PID open mark£¬AUTOMATIC¼°MANUAL
	bool tuning;			// auto tuning mark
	bool iMark;			 
	bool iTuneMark ;		// autotuner complete mark
	unsigned int iAutoTimes;	
	unsigned int iLastTimes;	
	double iSumOutput;
	unsigned int iTotalTimes;
	unsigned int iSinTimes;  	
	unsigned int iOKTimes;	
}TEMP_PID_STRUCT;

/**********************************º¯Êý***************************/
/*------------------------------------------------------------------------------
*Function: PID_Operation
* Brief: PID Control process 
* Param: sTemp	IN	 PID instance
* Retval:  ErrrorCode
*		:  bit0  stabilization
*          bit1  set SP mark
*		   bit2  CV out of range
* 		   (1:OK  0:OK)
----------------------------------------------------------------------------------*/
int PID_Operation(TEMP_PID_STRUCT *sTemp);

/*------------------------------------------------------------------------------
*Function: setupPID
* Brief: initial PID
* Param: 
*		sTemp	IN	 instance
*		iCurVal	IN	 PV
*		iSetVal	IN	 SP
*		iOutput	IN	 CV
*		iCurTime	IN	 system time
* Retval:  none
----------------------------------------------------------------------------------*/
void setupPID(TEMP_PID_STRUCT *sTemp, double *iCurVal, double *iSetVal, double *iOutput,
	unsigned int *iCurTime);

/*------------------------------------------------------------------------------
*Function: PID_Operation
* Brief: Set control parameter
* Param: 
*		sTemp	IN	 instance
*		sParm	IN	 control param
* Retval:  none
----------------------------------------------------------------------------------*/
void setupParm(TEMP_PID_STRUCT *sTemp, PID_INIT_PARM sParm);

