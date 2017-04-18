/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Run.c
*Note: This file is used for PID process (Combine with PID control process and autotune process)
----------------------------------------------------------------------------------------*/
#include "../include/PIDRun.h"

int iErrorCodeFYP = 0;
int iErrorCodeR1 = 0;

unsigned int Current_time;
// instance 1
TEMP_PID_STRUCT sFYP = { 0 };
PID_PARM_THREE sFYP_PARM = { 0 };
double dFYP_MaxOutPut = 0.0;
// instance 2
TEMP_PID_STRUCT sR1 = { 0 };
PID_PARM_THREE sR1_PARM = { 0 };
double dR1_MaxOutPut = 0.0;

// initial instance 1
void PID_Setup_FYP(void)
{
	PID_INIT_PARM sInitParmFYP = { 0 };
	// CV limit
	dFYP_MaxOutPut = 255;
	sFYP_PARM.dCurVal = 0;
	sFYP_PARM.dOutPut = 0;
	// set SP is 2000, it can be any value for necessary
	sFYP_PARM.dSetVal = 2000;
	sInitParmFYP.dMin = 0;
	sInitParmFYP.dMax = dFYP_MaxOutPut;
	// change sample time to 150ms
	sInitParmFYP.iSampleTimeNor = 150;
	sInitParmFYP.iDirection = DIRECT;
	// change KP/KI/KD
	sInitParmFYP.KP = 0.05 * sInitParmFYP.dMax;
	sInitParmFYP.KI = 0.01;
	sInitParmFYP.KD = 0;
	sInitParmFYP.iMode = AUTOMATIC;
	sInitParmFYP.aTuneLookBack = 5;
	sInitParmFYP.aTuneNoise = 5;
	sInitParmFYP.aTuneStartValue = 0;
	sInitParmFYP.aTuneStep = 0;
	sInitParmFYP.iType = 1;
	// 3 mimus  then  begin auto process
	sInitParmFYP.iSinTimes = 180000;
	sInitParmFYP.iOKTimes = 120000;
	setupPID(&sFYP, &(sFYP_PARM.dCurVal), &(sFYP_PARM.dSetVal), &(sFYP_PARM.dOutPut), &Current_time);
	setupParm(&sFYP, sInitParmFYP);
}

// initial instance 1
void PID_Setup_R1(void)
{
	PID_INIT_PARM sInitParmR1 = { 0 };
	dR1_MaxOutPut = 255;
	sR1_PARM.dCurVal = 0;
	sR1_PARM.dOutPut = 0;
	sR1_PARM.dSetVal = 2000;
	sInitParmR1.dMin = 0;
	sInitParmR1.dMax = dR1_MaxOutPut;
	sInitParmR1.iSampleTimeNor = 50;
	sInitParmR1.iDirection = DIRECT;
	sInitParmR1.KP = 0.05 * sInitParmR1.dMax;
	sInitParmR1.KI = 0;
	sInitParmR1.KD = 0;
	sInitParmR1.iMode = AUTOMATIC;
	sInitParmR1.aTuneLookBack = 3;
	sInitParmR1.aTuneNoise = 50;
	sInitParmR1.aTuneStartValue = 0;
	sInitParmR1.aTuneStep = 0;
	sInitParmR1.iType = 0;
	sInitParmR1.iSinTimes = 30000;
	sInitParmR1.iOKTimes = 1000;
	setupPID(&sR1, &(sR1_PARM.dCurVal), &(sR1_PARM.dSetVal), &(sR1_PARM.dOutPut), &Current_time);
	setupParm(&sR1, sInitParmR1);
}


int PID_Cal(TEMP_PID_STRUCT *sTemp, double dSetVal, double dCurVal)
{
	int iErrorCode = 0;
	*(sTemp->inputRun) = dCurVal;
	*(sTemp->setpointRun) = dSetVal;
	iErrorCode =  PID_Operation(sTemp);
	return iErrorCode;
}


//*********************END*********************//
