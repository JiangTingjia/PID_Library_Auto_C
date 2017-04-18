/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Nor.h
*Note: 此文件完成PID调节的必要功能，不需要修改
----------------------------------------------------------------------------------------*/
#include "../include/PID_Nor.h"

/**********************************函数***************************/
void PID(double* Input, double* Output, double* Setpoint, unsigned int *iCurTime, TEMP_NOR_STRUCT *sTempNor)
{
	// 设定输出限制
	double outMin = 0;
	double outMax = 255;
	unsigned int SampleTime = 100;	
	int ControllerDirection = DIRECT;
	double Kp = outMax * 0.01;
	double Ki = 0;
	double Kd = 0;
	int Mode = AUTOMATIC;
	sTempNor->Current_time = iCurTime;
	sTempNor->myOutput = Output;	// 初始化控制量
	sTempNor->myInput = Input;	//初始化被控制量
	sTempNor->mySetpoint = Setpoint;	//初始化设定值
	sTempNor->inAuto = false;		// 先关闭PID调节
	SetOutputLimits(outMin, outMax, sTempNor);				//默认PID限制为0到255
	// 默认采样周期为100ms
	SetSampleTime(SampleTime, sTempNor);
	SetControllerDirection(ControllerDirection, sTempNor); // 设置输出的方向
	SetTunings(Kp, Ki, Kd, sTempNor);			// 设置PID 控制参数
	SetMode(Mode, sTempNor);		// 设定PID控制开启
	sTempNor->lastTimeNor = *(sTempNor->Current_time) - sTempNor->SampleTime; 		// 获取初始化PID对应的系统运行时间
}

bool Compute(TEMP_NOR_STRUCT *sTempNor)
{
	unsigned int now = 0;		// 用于存储进入计算函数对应的系统运行时间
	unsigned int timeChange = 0;	// 用于储存此次进入计算的时间变化
	
	if (!sTempNor->inAuto) 	// 如果关闭PID，直接退出
	{
		return false;	
	}		
	
	now = *(sTempNor->Current_time);		// 计算到目前为止，过了多少ms
	timeChange = (now - sTempNor->lastTimeNor);
	
	if (timeChange >= sTempNor->SampleTime)		//如果所流逝的时间大于或者等于采样时间，那么则计算, 否则不满足采样条件，直接退出
	{
		double dInput = 0.0;
		double output = 0.0;
		double input = *(sTempNor->myInput);		//复制被控量给临时变量
		double error = *(sTempNor->mySetpoint) - input;	// 偏移量
		sTempNor->ITerm += (sTempNor->kiNor * error);		//计算积分项
		// 确保不超限，超限赋值为极限值
		if (sTempNor->ITerm > sTempNor->outMax) sTempNor->ITerm = sTempNor->outMax;
		else if (sTempNor->ITerm < sTempNor->outMin) sTempNor->ITerm = sTempNor->outMin;
		dInput = (input - sTempNor->lastInput);	//微分项 
		output = sTempNor->kpNor * error + sTempNor->ITerm - sTempNor->kdNor * dInput;	//PID 计算输出
		// 确保不超限，超限赋值为极限值
		if (output > sTempNor->outMax) output = sTempNor->outMax;
		else if (output < sTempNor->outMin) output = sTempNor->outMin;
		*(sTempNor->myOutput) = output;		//获取输出
		sTempNor->lastInput = input;
		sTempNor->lastTimeNor = now;		//更新计算时间
		return true;
	}
	else 
	{
		return false;
	}
}

void SetTunings(double Kp, double Ki, double Kd, TEMP_NOR_STRUCT *sTempNor)
{
	double SampleTimeInSec = 0.0;			// 将采样时间转换为s
	
	if (Kp < 0 || Ki < 0 || Kd < 0)  // PID调节参数不能为负
	{
		return;
	}
	
	sTempNor->dispKp = Kp; sTempNor->dispKi = Ki; sTempNor->dispKd = Kd; 		// 用于显示PID参数
	SampleTimeInSec = ((double)sTempNor->SampleTime) / 1000;
	// 调整PID参数，这个是为了消除改变参数时带来的突变，具体原理注释说不清楚
	sTempNor->kpNor = Kp;
	sTempNor->kiNor = Ki * SampleTimeInSec;
	sTempNor->kdNor = Kd / SampleTimeInSec;

	//	设定方向
	if (sTempNor->controllerDirection == REVERSE)
	{
		sTempNor->kpNor = (0 - sTempNor->kpNor);
		sTempNor->kiNor = (0 - sTempNor->kiNor);
		sTempNor->kdNor = (0 - sTempNor->kdNor);
	}
}

void SetSampleTime(unsigned int NewSampleTime, TEMP_NOR_STRUCT *sTempNor)
{
	// 改变采样时间
	if (NewSampleTime > 0)
	{
		double ratio = (double)NewSampleTime
			/ (double)sTempNor->SampleTime;
		sTempNor->kiNor *= ratio;
		sTempNor->kdNor /= ratio;
		sTempNor->SampleTime = (unsigned int)NewSampleTime;
	}
}

void SetOutputLimits(double Min, double Max, TEMP_NOR_STRUCT *sTempNor)
{
	if (Min >= Max) 
	{
		return;
	}
	sTempNor->outMin = Min;
	sTempNor->outMax = Max;

	if (sTempNor->inAuto)
	{
		if (*(sTempNor->myOutput) > sTempNor->outMax)
		{
			*(sTempNor->myOutput) = sTempNor->outMax;
		}
		else if (*(sTempNor->myOutput) < sTempNor->outMin)
		{
			*(sTempNor->myOutput) = sTempNor->outMin;
		}

		if (sTempNor->ITerm > sTempNor->outMax)
		{
			sTempNor->ITerm = sTempNor->outMax;
		}
		else if (sTempNor->ITerm < sTempNor->outMin)
		{
			sTempNor->ITerm = sTempNor->outMin;
		}
	}
}

void SetMode(int Mode, TEMP_NOR_STRUCT *sTempNor)
{
	bool newAuto = (Mode == AUTOMATIC);
	
	if (newAuto == !sTempNor->inAuto)	// 如果模式不一样，那么则重新初始化
	{  
		Initialize(sTempNor);
	}
	sTempNor->inAuto = newAuto;
}

void Initialize(TEMP_NOR_STRUCT *sTempNor)
{
	TEMP_NOR_STRUCT sTmp = *sTempNor;
	sTempNor->ITerm = *(sTempNor->myOutput);
	sTempNor->lastInput = *(sTempNor->myInput);
	if (sTempNor->ITerm > sTempNor->outMax)
	{
		sTempNor->ITerm = sTempNor->outMax;
	}
	else if (sTempNor->ITerm < sTempNor->outMin)
	{
		sTempNor->ITerm = sTempNor->outMin;
	}
	*sTempNor = sTmp;
}

void SetControllerDirection(bool Direction, TEMP_NOR_STRUCT *sTempNor)
{
	TEMP_NOR_STRUCT sTmp = *sTempNor;
	if (sTempNor->inAuto && Direction != sTempNor->controllerDirection)
	{
		sTempNor->kpNor = (0 - sTempNor->kpNor);
		sTempNor->kiNor = (0 - sTempNor->kiNor);
		sTempNor->kdNor = (0 - sTempNor->kdNor);
	}
	sTempNor->controllerDirection = Direction;
	*sTempNor = sTmp;
}

double GetKpNor(TEMP_NOR_STRUCT *sTempNor)
{ 
	return  sTempNor->dispKp;
}
double GetKiNor(TEMP_NOR_STRUCT *sTempNor)
{ 
	return  sTempNor->dispKi;
}
double GetKdNor(TEMP_NOR_STRUCT *sTempNor)
{ 
	return  sTempNor->dispKd;
}
int GetMode(TEMP_NOR_STRUCT *sTempNor)
{ 
	// 如果关闭，则关闭，如果其他，则开启
	return  sTempNor->inAuto ? AUTOMATIC : MANUAL;
}
int GetDirection(TEMP_NOR_STRUCT *sTempNor)
{ 
	return sTempNor->controllerDirection;
}

//**************ENDFILE****************//
