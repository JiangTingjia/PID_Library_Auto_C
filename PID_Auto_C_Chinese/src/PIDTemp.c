/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PIDTemp.c
*Note: 此文件完成PID调节的控制流程设计
----------------------------------------------------------------------------------------*/
#include "../include/PIDTemp.h"
#include "math.h"

// 改变PID参数
void setupParm(TEMP_PID_STRUCT *sTemp, PID_INIT_PARM sParm)
{
	//普通PID参数
	SetControllerDirection(sParm.iDirection, &(sTemp->sTempNor));		// 设定调节方向
	SetOutputLimits(sParm.dMin, sParm.dMax, &(sTemp->sTempNor));		// 设定控制量的范围
	SetSampleTime(sParm.iSampleTimeNor, &(sTemp->sTempNor));		//设定采样时间
	SetTunings(sParm.KP, sParm.KI, sParm.KD, &(sTemp->sTempNor));			// 设定调谐参数
	SetMode(sParm.iMode, &(sTemp->sTempNor));		// 设定PID控制开启
	// 自动调谐参数
	SetControlType(sParm.iType, &(sTemp->sTempAuto));			// 设定PI控制还是PID控制
	SetNoiseBand(sParm.aTuneNoise, &(sTemp->sTempAuto));		//设定噪声阈值
	SetOutputStep(sParm.aTuneStep, &(sTemp->sTempAuto));		//设定PWM半峰值
	SetLookbackSec((int)(sParm.aTuneLookBack), &(sTemp->sTempAuto));	//设定回溯数量
	// 设定震荡与稳定时间
	sTemp->iSinTimes = sParm.iSinTimes;
	sTemp->iOKTimes = sParm.iOKTimes;
}

// 初始化PID调节器,初始化变量
void setupPID(TEMP_PID_STRUCT *sTemp, double *iCurVal, double *iSetVal, double *iOutput,
	unsigned int *iCurTime)
{
	sTemp->inputRun = iCurVal;
	sTemp->setpointRun = iSetVal;
	sTemp->outputRun = iOutput;
	// 初始化普通PID
	PID(sTemp->inputRun, sTemp->outputRun, sTemp->setpointRun, iCurTime, &(sTemp->sTempNor));		//初始化PID调节器，正向调节
	PID_ATune(sTemp->inputRun, sTemp->outputRun, sTemp->setpointRun, iCurTime, &(sTemp->sTempAuto));
}

//用于自动调谐结束后，恢复普通PID控制状态
void AutoTuneHelper(int start, TEMP_PID_STRUCT *sTemp)
{
	if (start)
	{
		// 返回原工作状态
		sTemp->ATuneModeRemember = GetMode(&(sTemp->sTempNor));
	}
	else
	{
		// 设定开启状态
		SetMode(sTemp->ATuneModeRemember, &(sTemp->sTempNor));
	}
}

// 从普通PID转换为自动调谐
void changeAutoTune(TEMP_PID_STRUCT *sTemp)
{
	if (!sTemp->tuning)
	{
		double StartVal = sTemp->iSumOutput / sTemp->iTotalTimes;
		SetStartOutput(StartVal, &(sTemp->sTempAuto));
		SetOutputStep(sTemp->sTempAuto.outputStart, &(sTemp->sTempAuto));
		//  记忆开启 自动调谐 PID控制前  PID控制器的工作状态
		AutoTuneHelper(true, sTemp);
		sTemp->tuning = true;
	}
	else
	{ 
		// 取消自动调谐 ，并回复原PID控制器工作状态
		Cancel(&(sTemp->sTempAuto));
		sTemp->tuning = false;
		AutoTuneHelper(false, sTemp);
	}
}

// 判断是否进入震荡
bool SinTune(TEMP_PID_STRUCT *sTemp)
{
	// 如果调谐时间为零0，那么就不进入自动调参
	if (sTemp->iSinTimes <= 0)
	{
		return false;
	}
	//当温度首次达到设定值后，5分钟后开始自动调谐
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

// 判断PID控制是否进入稳定
bool PID_OK(TEMP_PID_STRUCT *sTemp)
{
	double DiffInput = 0.0;
	// 如果iOKTimes为零，那么不判断是否满足噪声阈值条件，认为用于都满足
	if (0 == sTemp->iOKTimes)
	{
		return true;
	}
	
	if (true == sTemp->iTuneMark)
	{
		DiffInput = fabs(*(sTemp->inputRun) - *(sTemp->setpointRun));
		sTemp->iAutoTimes = *(sTemp->sTempNor.Current_time);
		// 连续n分钟都小于噪声阈值，则稳定
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

//  PID计算函数
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
		// 调用自动调谐运行函数，返回标志位， 非零表示成功
		int val = 0;
		val = (Runtime(&(sTemp->sTempAuto)));
		// 如果成功，关闭自动调谐
		if (val != 0)
		{
			sTemp->iLastTimes = *(sTemp->sTempNor.Current_time);
			sTemp->iAutoTimes = 0;
			sTemp->iTuneMark = true;  // JTJ
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
		// 判断是否进入震荡，并持续执行PID控制
		if (false == sTemp->iTuneMark)
		{
			sTemp->tuning = SinTune(sTemp);
			// 设定自动调谐参数
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
	
	// 赋值占空比
	*(sTemp->outputRun) = (unsigned int)*(sTemp->outputRun);

	if (*(sTemp->outputRun) < 0 || *(sTemp->inputRun) < 0 || *(sTemp->setpointRun)< 0)
	{
		*(sTemp->outputRun) = sTemp->sTempNor.outMin;
	}
	
	// 进入稳定期，可以改变设定值
	if(true == bOK)
	{
		iErrorCode = 0x00;
		return iErrorCode;
	}
	if( true == sTemp->iTuneMark)	// 自动调参期结束， 都可以改变设定值
	{
		iErrorCode = 0x01;
		return iErrorCode;
	}
	if(true == sTemp->tuning)	// 自动调参期，不可改变
	{
		iErrorCode = 0x03;
		return iErrorCode;
	}
	if(true == sTemp->iMark)	// 震荡期，不可改变
	{
		iErrorCode = 0x03;
		return iErrorCode;
	}
	iErrorCode = 0x01;
	return iErrorCode;
}
