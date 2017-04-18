/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PIDRun.c
*Note: 此文件完成初始化函数，需要修改初始化实例中必要参数
----------------------------------------------------------------------------------------*/
#include "../include/PIDRun.h"
// 错误码 第一位代表 是否稳定； 第二位代表是否可以改变设定值：  第三位代表温度是否异常
// 0:表示稳定、可以改变、无异常 1：表示未稳定、不可改变、温度异常
int iErrorCodeFYP = 0;
int iErrorCodeR1 = 0;

//系统时间 ms
unsigned int Current_time;
// 实例1
TEMP_PID_STRUCT sFYP = { 0 };
PID_PARM_THREE sFYP_PARM = { 0 };
double dFYP_MaxOutPut = 0.0;
// 实例2
TEMP_PID_STRUCT sR1 = { 0 };
PID_PARM_THREE sR1_PARM = { 0 };
double dR1_MaxOutPut = 0.0;

// 初始化实例1
void PID_Setup_FYP(void)
{
	PID_INIT_PARM sInitParmFYP = { 0 };
	// 此处可由大家自由配置
	{
		// 设定实例1对应的PWM输出上限
		dFYP_MaxOutPut = 255;
		sInitParmFYP.dMin = 0;
		// 设定PID计算周期
		sInitParmFYP.iSampleTimeNor = 150;
		sInitParmFYP.iDirection = DIRECT;
		// 自动调参阈值，如果系统敏感，则适宜将此设定值设大
		// 例如稍微增加一点输出，PV会迅速变化，那么需要将此值设大，此处可以设定为4倍的传感器精度
		sInitParmFYP.aTuneNoise = 5;
		sInitParmFYP.iType = 1;
		// 在开机后3分钟后进入自动调参，如果系统很灵敏，那么可以适当减小此参数，如果此参数设置为0，那么系统就不会进入自动调参
		sInitParmFYP.iSinTimes = 180000;
		// 自动调参结束后连续2分钟 PV在阈值范围内，则表示稳定，如果设置为0，那么用于认为系统稳定
		sInitParmFYP.iOKTimes = 120000;
	}

	// 其中dCurVal 和 dSetVal这里没有意义，通过PID_Cal接口给出
	sFYP_PARM.dCurVal = 0;
	sFYP_PARM.dOutPut = 0;
	sFYP_PARM.dSetVal = 2000;
	sInitParmFYP.dMax = dFYP_MaxOutPut;

	// 设定初始的KP/KI/KD
	sInitParmFYP.KP = 0.05 * sInitParmFYP.dMax;
	sInitParmFYP.KI = 0.01;
	sInitParmFYP.KD = 0;
	// 开启PID控制标志
	sInitParmFYP.iMode = AUTOMATIC;
	// 这个参数尽量不要修改
	sInitParmFYP.aTuneLookBack = 5;
	sInitParmFYP.aTuneStartValue = 0;
	sInitParmFYP.aTuneStep = 0;
	setupPID(&sFYP, &(sFYP_PARM.dCurVal), &(sFYP_PARM.dSetVal), &(sFYP_PARM.dOutPut), &Current_time);
	setupParm(&sFYP, sInitParmFYP);
}

// 初始化实例2
void PID_Setup_R1(void)
{
	PID_INIT_PARM sInitParmR1 = { 0 };
	// 输出上限
	dR1_MaxOutPut = 255;
	sR1_PARM.dCurVal = 0;
	sR1_PARM.dOutPut = 0;
	// 设定值
	sR1_PARM.dSetVal = 2000;
	sInitParmR1.dMin = 0;
	sInitParmR1.dMax = dR1_MaxOutPut;
	// PID计算周期
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
