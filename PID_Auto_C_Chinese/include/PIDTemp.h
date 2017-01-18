/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PIDTemp.h
*Note: 此文件完成PID调节的控制流程设计
----------------------------------------------------------------------------------------*/
#ifndef PID_TEMP_H
#define PID_TEMP_H
#include "PID_Nor.h"
#include "PID_Auto.h"

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
	// 用于判断是否进入震荡
	unsigned int iSinTimes;  	// MS
	// 用于判断是否稳定
	unsigned int iOKTimes;		// MS
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
	bool iTuneMark ;		// 自动调谐是否结束标志
	// 震荡及稳定判断标志和参数
	unsigned int iAutoTimes;	// 用于判断是进入震荡
	unsigned int iLastTimes;	// 用于判断是进入震荡
	double iSumOutput;
	unsigned int iTotalTimes;
	unsigned int iSinTimes;  	// MS
	unsigned int iOKTimes;		// MS
}TEMP_PID_STRUCT;

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
int PID_Operation(TEMP_PID_STRUCT *sTemp);

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
	unsigned int *iCurTime);

/*------------------------------------------------------------------------------
*Function: PID_Operation
* Brief: 此函数用于调节PID控制的流程
* Param: 
*		sTemp	IN	 PID控制对象实例
*		sParm	IN	 PID参数

* Retval:  无
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void setupParm(TEMP_PID_STRUCT *sTemp, PID_INIT_PARM sParm);

#endif

