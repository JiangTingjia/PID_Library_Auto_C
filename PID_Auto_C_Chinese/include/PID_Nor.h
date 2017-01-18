/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Nor.h
*Note: 此文件完成PID调节的必要功能，不需要修改
----------------------------------------------------------------------------------------*/
#ifndef PID_Nor_h
#define PID_Nor_h
#include "stdbool.h"

#define AUTOMATIC	1		//开启PID调节
#define MANUAL	0			//关闭PID调节
#define DIRECT  0			//正向调节
#define REVERSE  1			//反向调节

///***********************************变量****************************/
typedef struct
{
	// 可设定的参数
	double kpNor;                  // Kp参数 (参与PID计算的实参数)
	double kiNor;                  // Ki参数 (参与PID计算的实参数)
	double kdNor;                  // Kd参数 (参与PID计算的实参数)
	bool controllerDirection;		  // 控制方向 0 正向
	unsigned int SampleTime;	// 采样时间 ms
	double outMin, outMax;		// 上下限
	bool inAuto;					// 是否开启PID控制标识 1开启
	// 运行过程中需要的控制参数
	double *myInput;              //  被控量
	double *myOutput;             //  控制量
	double *mySetpoint;           //  设定值 
	unsigned int *Current_time;		//系统当前运行时间
	unsigned int lastTimeNor;	 //  上一次计算时对应的系统运行时间
	double ITerm;				// 积分项
	double lastInput;			// 上一次被控量
	// 标志位
	double dispKp;				// Kp参数 (Get参数时返回值)
	double dispKi;				// Ki参数 (Get参数时返回值)
	double dispKd;				// Kd参数 (Get参数时返回值)
}TEMP_NOR_STRUCT;

/**********************************函数***************************/
/*------------------------------------------------------------------------------
*Function: PID
* Brief: 此函数用于初始化PID调节器
* Param: Input  IN  被控制量
*        Output  IN  控制量
*        Setpoint  IN  设定值
*        Kp  IN  P参数
*        Ki  IN  I参数
*        Kd  IN  D参数
*        ControllerDirection  IN  调节器的工作方向
* Retval:  无返回值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void PID(double* Input, double* Output, double* Setpoint, unsigned int *iCurTime, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: Initialize
* Brief: 此函数用于修改PID采样时间后，参数重新初始化，外部不需要调用
* Param: 无
* Retval: 无
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void Initialize(TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: Compute
* Brief: 此函数用于计算PID控制器的输出，也就是控制量
* Param: 无
* Retval:  成功：true
*		   失败：false
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
bool Compute(TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetMode
* Brief: 此函数用于开启或者关闭PID调节 AUTOMATIC or MANUAL
* Param: Mode  IN  PID控制器开关
* Retval:  无返回值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetMode(int Mode, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: GetMode
* Brief: 此函数用于获取PID控制器 是否开启
* Param: 无
* Retval: 开启: 1
*         关闭：0
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
int GetMode(TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetOutputLimits
* Brief: 此函数用于设定PID控制器输出上下限， 默认为0到255
* Param: Min  IN  下限
*        Max  IN  上限
* Retval:  无
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetOutputLimits(double Min, double Max, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetSampleTime
* Brief: 此函数用于设定PID控制器 采样频率 ， 默认为100ms
* Param: NewSampleTime  IN  采样频率(ms)
* Retval:  无
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetSampleTime(unsigned int NewSampleTime, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetTunings
* Brief: 此函数用于设定PID控制器 P I D参数
* Param: Kp  IN  P参数
*        Ki  IN  I参数
*        Kd  IN  D参数
* Retval:  无
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetTunings(double Kp, double Ki, double Kd, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: GetKpNor
* Brief: 此函数用于获取PID控制器 当前P参数
* Param: 无
* Retval:  P参数值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
double GetKpNor(TEMP_NOR_STRUCT *sTempNor);						  // These functions query the pid for interal values.

/*------------------------------------------------------------------------------
*Function: GetKiNor
* Brief: 此函数用于获取PID控制器 当前I参数
* Param: 无
* Retval:  I参数值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
double GetKiNor(TEMP_NOR_STRUCT *sTempNor);						  //  they were created mainly for the pid front-end,

/*------------------------------------------------------------------------------
*Function: GetKdNor
* Brief: 此函数用于获取PID控制器 当前D参数
* Param: 无
* Retval:  D参数值
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
double GetKdNor(TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetControllerDirection
* Brief: 此函数用于设定PID控制器 输出方向
* Param: Direction  IN  方向宏
* Retval:  无
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetControllerDirection(bool Direction, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: GetDirection
* Brief: 此函数用于获取PID控制器 方向
* Param: 无
* Retval: 正向: 0
*         反向：1
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
int GetDirection(TEMP_NOR_STRUCT *sTempNor);

#endif

