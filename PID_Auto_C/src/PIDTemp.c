/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PIDTemp.c
*Note: This file is used for PID Autoning
----------------------------------------------------------------------------------------*/
#include "../include/PIDTemp.h"
#include "math.h"

// change some param
void setupParm(TEMP_PID_STRUCT *sTemp, PID_INIT_PARM sParm)
{
	// PID 
	SetControllerDirection(sParm.iDirection, &(sTemp->sTempNor));		
	SetOutputLimits(sParm.dMin, sParm.dMax, &(sTemp->sTempNor));		
	SetSampleTime(sParm.iSampleTimeNor, &(sTemp->sTempNor));		
	SetTunings(sParm.KP, sParm.KI, sParm.KD, &(sTemp->sTempNor));			
	SetMode(sParm.iMode, &(sTemp->sTempNor));		
	// auto
	SetControlType(sParm.iType, &(sTemp->sTempAuto));			
	SetNoiseBand(sParm.aTuneNoise, &(sTemp->sTempAuto));		
	SetOutputStep(sParm.aTuneStep, &(sTemp->sTempAuto));		
	SetLookbackSec((int)(sParm.aTuneLookBack), &(sTemp->sTempAuto));	
	// time
	sTemp->iSinTimes = sParm.iSinTimes;
	sTemp->iOKTimes = sParm.iOKTimes;
}

void setupPID(TEMP_PID_STRUCT *sTemp, double *iCurVal, double *iSetVal, double *iOutput,
	unsigned int *iCurTime)
{
	sTemp->inputRun = iCurVal;
	sTemp->setpointRun = iSetVal;
	sTemp->outputRun = iOutput;
	PID(sTemp->inputRun, sTemp->outputRun, sTemp->setpointRun, iCurTime, &(sTemp->sTempNor));		
	PID_ATune(sTemp->inputRun, sTemp->outputRun, sTemp->setpointRun, iCurTime, &(sTemp->sTempAuto));
}

void AutoTuneHelper(int start, TEMP_PID_STRUCT *sTemp)
{
	if (start)
	{
		sTemp->ATuneModeRemember = GetMode(&(sTemp->sTempNor));
	}
	else
	{
		SetMode(sTemp->ATuneModeRemember, &(sTemp->sTempNor));
	}
}

void changeAutoTune(TEMP_PID_STRUCT *sTemp)
{
	if (!sTemp->tuning)
	{
		double StartVal = sTemp->iSumOutput / sTemp->iTotalTimes;
		SetStartOutput(StartVal, &(sTemp->sTempAuto));
		SetOutputStep(sTemp->sTempAuto.outputStart, &(sTemp->sTempAuto));
		AutoTuneHelper(true, sTemp);
		sTemp->tuning = true;
	}
	else
	{ 
		Cancel(&(sTemp->sTempAuto));
		sTemp->tuning = false;
		AutoTuneHelper(false, sTemp);
	}
}

// oscillate
bool SinTune(TEMP_PID_STRUCT *sTemp)
{
	// 如果调谐时间为零0，那么就不进入自动调参
	if (sTemp->iSinTimes <= 0)
	{
		return false;
	}
	double DiffInput = fabs(*(sTemp->inputRun) - *(sTemp->setpointRun));
	sTemp->iAutoTimes = *(sTemp->sTempNor.Current_time);
	if (false == sTemp->iMark && DiffInput <=  sTemp->sTempAuto.noiseBand && *(sTemp->sTempNor.Current_time) > (sTemp->iSinTimes) / 3)
	{
		sTemp->iLastTimes = *(sTemp->sTempNor.Current_time);
		sTemp->iMark = true;
	}
	if (true == sTemp->iMark)
	{
		sTemp->iTotalTimes = sTemp->iTotalTimes + 1;
		sTemp->iSumOutput = sTemp->iSumOutput + *(sTemp->outputRun);
		if (sTemp->iAutoTimes - sTemp->iLastTimes > sTemp->iSinTimes && DiffInput <=  sTemp->sTempAuto.noiseBand)
		{
			sTemp->iMark = false;
			return true;
		}
	}
	return false;
}

// stablization
bool PID_OK(TEMP_PID_STRUCT *sTemp)
{
	double DiffInput = 0.0;
	if (0 == sTemp->iOKTimes)
	{
		return true;
	}
	if (true == sTemp->iTuneMark)
	{
		DiffInput = fabs(*(sTemp->inputRun) - *(sTemp->setpointRun));
		sTemp->iAutoTimes = *(sTemp->sTempNor.Current_time);
		if (false == sTemp->iMark && DiffInput <= sTemp->sTempAuto.noiseBand)
		{
			sTemp->iMark = true;
		}
		else if(DiffInput > sTemp->sTempAuto.noiseBand)
		{
			sTemp->iMark = false;
			sTemp->iLastTimes = *(sTemp->sTempNor.Current_time);
		}
		
		if (sTemp->iAutoTimes - sTemp->iLastTimes > sTemp->iOKTimes && true == sTemp->iMark)
		{
			return true;
		}
	}
	return false;
}

//  PID control
int PID_Operation(TEMP_PID_STRUCT *sTemp)
{
	int iErrorCode = 0x05;
	bool bOK = false;
	double Diff = 0.0;
	Diff = *(sTemp->inputRun) - *(sTemp->setpointRun);
	// 不输出
	if(Diff >= *(sTemp->setpointRun) / 2)
	{
		sTemp->iMark = false;
		iErrorCode = 0x05;
		*(sTemp->outputRun) = sTemp->sTempNor.outMin;
		return iErrorCode;
	}

	if (sTemp->tuning)
	{
		int val = 0;
		val = (Runtime(&(sTemp->sTempAuto)));
		if (val != 0)
		{
			sTemp->iLastTimes = *(sTemp->sTempNor.Current_time);
			sTemp->iAutoTimes = 0;
			sTemp->iTuneMark = true;  
			sTemp->tuning = false;
		}
		if (!sTemp->tuning)
		{ 
			double KP = GetKpAuto(&(sTemp->sTempAuto));
			double KI = GetKiAuto(&(sTemp->sTempAuto));
			double KD = GetKdAuto(&(sTemp->sTempAuto));
			SetTunings(KP, KI, KD, &(sTemp->sTempNor));
			AutoTuneHelper(false, sTemp);
		}
	}
	else
	{
		if (false == sTemp->iTuneMark)
		{
			sTemp->tuning = SinTune(sTemp);
			if (sTemp->tuning)
			{
				sTemp->tuning = false;
				changeAutoTune(sTemp);
				sTemp->tuning = true;
			}
		}
		Compute(&(sTemp->sTempNor));
		bOK =  PID_OK(sTemp);
	}
	
	*(sTemp->outputRun) = (unsigned int)*(sTemp->outputRun);

	if (*(sTemp->outputRun) < 0 || *(sTemp->inputRun) < 0 || *(sTemp->setpointRun)< 0)
	{
		*(sTemp->outputRun) = sTemp->sTempNor.outMin;
	}
	
	if(true == bOK)
	{
		iErrorCode = 0x00;
		return iErrorCode;
	}
	if( true == sTemp->iTuneMark)	
	{
		iErrorCode = 0x01;
		return iErrorCode;
	}
	if(true == sTemp->tuning)	
	{
		iErrorCode = 0x03;
		return iErrorCode;
	}
	if(true == sTemp->iMark)	
	{
		iErrorCode = 0x03;
		return iErrorCode;
	}
	iErrorCode = 0x01;
	return iErrorCode;
}
