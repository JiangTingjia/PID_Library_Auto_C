#include "include\PID_Nor.h"
#include "include\PID_Auto.h"
#include <stdbool.h>

typedef struct
{
	// 普通PID控制参数
	double KP;			//Kp参数
	double KI;			//Ki参数
	double KD;			//Kd参数
	double dMin;		// 上限
	double dMax;		// 下限
	unsigned int iSampleTimeNor;		// PID调节采样时间
	int iDirection;						// PID调节方向
	int iMode;							//PID开启开关
	//自动调谐参数
	double aTuneStep;					// 自动调参PWM波半峰值
	double aTuneNoise;					//噪声阈值
	double aTuneStartValue;				// 自动调谐输出基准值（暂时不使用，留作备用）
	unsigned int aTuneLookBack;		//回溯时间
	int iType;						// PI控制 0   PID控制 1
}PID_INIT_PARM;

typedef struct
{
	// 控制参数
	double *inputRun;		// 输入		
	double *outputRun;		// 输出
	double *setpointRun;	// 设定值
	TEMP_NOR_STRUCT sTempNor;	// 普通PID控制参数
	TEMP_AUTO_STRUCT sTempAuto;	//自动调谐控制参数
								// 判断参数
	int ATuneModeRemember;	// PID开启标志，AUTOMATIC及MANUAL
	bool tuning;			// 自动调谐开启标志位
	bool iMark;			//  判断振荡标志位
	bool iTuneMark;		// 自动调谐是否结束标志
}TEMP_PID_STRUCT;

int PID_Operation(TEMP_PID_STRUCT *sTemp);
void setupPID(TEMP_PID_STRUCT *sTemp, double *iCurVal, double *iSetVal, double *iOutput,
	unsigned int *iCurTime);
void setupParm(TEMP_PID_STRUCT *sTemp, PID_INIT_PARM sParm);


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
		//*****************************注意*********************//
		// 这里的StartVal最好设定为进入震荡期后2个震荡周期的 output的均值, Step = StartVal
		// 自动调参期间，output会在 StartVal - Step 和 StartVal + Step,如果这两个参数选取不合适，input会发散
		double StartVal = *(sTemp->sTempNor.myOutput);
		SetStartOutput(StartVal, &(sTemp->sTempAuto));
		// 此处为自动调参的半峰值设定
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

/**********************************函数***************************/
/*------------------------------------------------------------------------------
*Function: PID_Operation
* Brief: 此函数用于调节PID控制的流程
* Param:
*		sTemp	IN	 PID控制对象实例
* Retval:  错误码
*		:  bit0  是否进入稳定区
*          bit1  是否可以改变设定值
*		   bit2  温度是否超过
* 		   (1表示不OK  0 表示OK)
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
int PID_Operation(TEMP_PID_STRUCT *sTemp)
{
	int iErrorCode = 0;
	bool bOK = false;
	double Diff = 0.0;
	Diff = *(sTemp->inputRun) - *(sTemp->setpointRun);

	// 如果系统没有进入震荡，那么执行Compute，进行正常的PID调节，也就是运行nor部分
	// 如果发现系统进入了震荡，那么将 sTemp->iTuneMark 置为 true， changeAutoTune函数为进入自动调参的初始化（进入震荡期目前是认为设定的，如果有合适的方法判断是否进入震荡，那么可修改成自动进入）
	// 等待调参结束后，系统会更行PID参数并继续运行nor的PID调节
	if (sTemp->tuning)
	{
		// 调用自动调谐运行函数，返回标志位， 非零表示成功
		int val = 0;
		val = (Runtime(&(sTemp->sTempAuto)));
		// 如果成功，关闭自动调谐
		if (val != 0)
		{
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
		// 
		if (false == sTemp->iTuneMark)
		{
			changeAutoTune(sTemp);
		}
		Compute(&(sTemp->sTempNor));
	}

	// 赋值占空比
	*(sTemp->outputRun) = (unsigned int)*(sTemp->outputRun);

	if (*(sTemp->outputRun) < 0 || *(sTemp->inputRun) < 0 || *(sTemp->setpointRun)< 0)
	{
		*(sTemp->outputRun) = sTemp->sTempNor.outMin;
	}

	// 进入稳定期，可以改变设定值
	if (true == bOK)
	{
		iErrorCode = 0x00;
		return iErrorCode;
	}
	if (true == sTemp->iTuneMark)	// 自动调参期结束， 都可以改变设定值
	{
		iErrorCode = 0x01;
		return iErrorCode;
	}
	if (true == sTemp->tuning)	// 自动调参期，不可改变
	{
		iErrorCode = 0x03;
		return iErrorCode;
	}
	if (true == sTemp->iMark)	// 震荡期，不可改变
	{
		iErrorCode = 0x03;
		return iErrorCode;
	}
	iErrorCode = 0x01;
	return iErrorCode;
}

/*------------------------------------------------------------------------------
*Function: setupPID
* Brief: 此函数初始化PID控制器
* Param:
*		sTemp	IN	 PID控制对象实例
*		iCurVal	IN	 输入值
*		iSetVal	IN	 设定值
*		iOutput	IN	 输出值
*		iCurTime	IN	 系统时间
* Retval:  无
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
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

/*------------------------------------------------------------------------------
*Function: PID_Operation
* Brief: 此函数用于调节PID控制的流程
* Param:
*		sTemp	IN	 PID控制对象实例
*		sParm	IN	 PID参数

* Retval:  无
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
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
	SetLookbackSec((int)(sParm.aTuneLookBack), &(sTemp->sTempAuto));	// 设定回溯数量
}