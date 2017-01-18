/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Auto.h
*Note: 此文件完成 PID 自动调参必要功能，不需要修改
----------------------------------------------------------------------------------------*/
#ifndef PID_Auto
#define PID_Auto

#include "stdbool.h"

typedef struct
{
	/***********************************变量****************************/
	// 可设定的变量
	double noiseBand;			// 噪声带宽
	int controlType;			// 运行模式标志， 1代表PID控制，其他代表PI控制
	int nLookBack;		// 回溯的数量
	double oStep;		// 表示输出PWM半峰值
	double outputStart;		// PWM 均值
	// 运行中所需变量
	unsigned int *Current_time;		//系统当前运行时间
	double *input, *output, *setpoint;		// 控制量和被控量、设定值
	bool isMax, isMin;	// 是否最大值和最小值的标志位
	bool running;				// 自动调参是否运行标志位
	unsigned int peak1, peak2, lastTimeAuto;	// 用于储存峰顶出现的时间
	unsigned int sampleTime;		// 采样时间
	int peakType;		// 三个值 -1 0 1 ； -1 代表峰谷， 0 代表未有峰值，  1代表峰顶
	double lastInputs[101];		// 用于存储被控量历史变化
	double peaks[10];		// 用于存储参考值
	int peakCount;		// 用于存储峰值数量
	bool justchanged;	// 是否存在峰值转换标志位
	bool justevaled;	// 是否开始计算标志位
	double absMax, absMin;	// 最大值及最小值的绝对值
	double Ku, Pu;	// PID参数的调谐因子
}TEMP_AUTO_STRUCT;

/**********************************函数***************************/
/*------------------------------------------------------------------------------
*Function: PID_ATune
* Brief: 此函数用于初始化自动调参PID调节器
* Param: Input  IN  被控制量
*        Output  IN  控制量
* Retval:  无返回值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void PID_ATune(double* Input, double* Output, double *setpoint, unsigned int *iCurTime, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: Runtime
* Brief: 此函数用于运行自动调谐计算
* Param: sTempAuto	IN 自动调谐参数实例
* Retval:  0：成功
*		   1：失败
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
int Runtime(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: Cancel
* Brief: 此函数用于取消自动调谐PID控制器运行
* Param: sTempAuto	IN 自动调谐参数实例
* Retval:  无返回值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void Cancel(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetOutputStep
* Brief: 此函数用于设定自动调谐输出的PWM半峰值
* Param: Step  IN  半峰值
*        sTempAuto	IN 自动调谐参数实例
* Retval:  无返回值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetOutputStep(double Step, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetOutputStep
* Brief: 此函数用于获取自动调谐输出的PWM半峰值
* Param:   sTempAuto	IN 自动调谐参数实例
* Retval:  半峰值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
double GetOutputStep(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetStartOutput
* Brief: 此函数用于设定PWM基准值
* Param: StartVal  IN  被控制量
*        sTempAuto	IN 自动调谐参数实例
* Retval:  无返回值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetStartOutput(double StartVal, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GettStartOutput
* Brief: 此函数用于获取PWM基准值
* Param: 
*        sTempAuto	IN 自动调谐参数实例
* Retval:  基准值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
double GettStartOutput(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetControlType
* Brief: 此函数用于设定控制模式
* Param: Type  IN  1表示PID控制，  0表示PI控制
*        sTempAuto	IN 自动调谐参数实例
* Retval:  无返回值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetControlType(int Type, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetControlType
* Brief: 此函数用于获取PID控制器控制模式
* Param:
*        sTempAuto	IN 自动调谐参数实例
* Retval:  0：PI
*		   1：PID
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
int GetControlType(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetLookbackSec
* Brief: 此函数用于设定回溯时间
* Param: value  IN  回溯时间
*        sTempAuto	IN 自动调谐参数实例
* Retval:  无返回值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetLookbackSec(int value, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetLookbackSec
* Brief: 此函数用于获取回溯时间
* Param:
*        sTempAuto	IN 自动调谐参数实例
* Retval:  回溯时间
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
int GetLookbackSec(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetNoiseBand
* Brief: 此函数用于设定噪声区间
* Param: Band  IN  噪声区间
*        sTempAuto	IN 自动调谐参数实例
* Retval:  无返回值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetNoiseBand(double Band, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetNoiseBand
* Brief: 此函数用于获取噪声区间
* Param:
*        sTempAuto	IN 自动调谐参数实例
* Retval:  噪声区间
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
double GetNoiseBand(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetKpAuto/GetKiAuto/GetKdAuto
* Brief: 此函数用于获取PID控制参数
* Param: 
*        sTempAuto	IN 自动调谐参数实例
* Retval:  Kp/kI/Kd
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
double GetKpAuto(TEMP_AUTO_STRUCT *sTempAuto);										
double GetKiAuto(TEMP_AUTO_STRUCT *sTempAuto);										
double GetKdAuto(TEMP_AUTO_STRUCT *sTempAuto);										

/*------------------------------------------------------------------------------
*Function: FinishUp
* Brief: 此函数用于调节结束后计算PID参数中间变量
* Param: 
*        sTempAuto	IN 自动调谐参数实例
* Retval:  无返回值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void FinishUp(TEMP_AUTO_STRUCT *sTempAuto);


#endif

