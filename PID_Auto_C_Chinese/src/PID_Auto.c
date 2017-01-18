/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Auto.c
*Note: 此文件完成 PID 自动调参必要功能，不需要修改
----------------------------------------------------------------------------------------*/
#include "PID_Auto.h"
#include "math.h"

/**********************************函数***************************/
void PID_ATune(double* Input, double* Output, double *setpoint, unsigned int *iCurTime, TEMP_AUTO_STRUCT *sTempAuto)
{
	int Type; 	//默认为PID控制
	double noiseBand;	// 默认噪声带宽
	double oStep;		// 默认PWM 半峰值
	int LookBack;		// 设定回溯数量
	double StartVal;	// 设定PWM输出基准值
	sTempAuto->Current_time = iCurTime;
	// 赋值控制量 被控量
	sTempAuto->input = Input;
	sTempAuto->output = Output;
	sTempAuto->setpoint = setpoint;
	Type = 0;				//默认PI控制
	SetControlType(Type, sTempAuto);
	noiseBand = *(sTempAuto->setpoint) * 0.02;	//默认噪声为设定值的2%
	SetNoiseBand(noiseBand, sTempAuto);
	oStep = *(sTempAuto->output);			//默认PWM半峰为当前输出
	SetOutputStep(oStep, sTempAuto);
	StartVal = *(sTempAuto->output);		//默认PWM基准值为当前输出
	SetStartOutput(StartVal, sTempAuto);
	LookBack = 10;							//默认回溯数量为10
	SetLookbackSec(LookBack, sTempAuto);	
	sTempAuto->running = false;	// 默认自动调谐为关闭
	sTempAuto->lastTimeAuto = *(sTempAuto->Current_time);
}

//	关闭 自动调谐
void Cancel(TEMP_AUTO_STRUCT *sTempAuto)
{
	sTempAuto->running = false;
}

//	计算调谐参数主控制程序
// 如果自动调谐无法退出，可以更改start 和 ostep来满足条件
int Runtime(TEMP_AUTO_STRUCT *sTempAuto)
{
	unsigned int now = 0;		// 用于存储 当前系统运行时间
	double refVal = 0.0;		//记录刚进入函数对应的被控量
	int i = 0;					//用于for循环的变量
	sTempAuto->justevaled = false;				// 表示目前不能计算PID参数
	// 如果已经经过的峰顶数量超过阈值，并且自动调谐开启，那么计算PID参数
	if (sTempAuto->peakCount > 5 && sTempAuto->running)
	{
		sTempAuto->running = false;	//将自动调谐关闭
		FinishUp(sTempAuto);		// 这个函数是用来计算PID参数调谐因子的
		return 1;
	}
	now = *(sTempAuto->Current_time);		// 获取当前时间，此时间主要是用来判断时间间隔是否超过采样间隔
	/***********************************************/
	//如果当前运行时间与初始化的时间差小于采样时间，那么函数退出，计算失败，也就是意味着，未满足采样间隔，不计算
	if ((now - sTempAuto->lastTimeAuto) < sTempAuto->sampleTime) 
	{
		return 0;
	}
	/***********************************************/
	sTempAuto->lastTimeAuto = now;		//获取此次计算对应的系统运行时间
	refVal = *(sTempAuto->input);		//获取此时的被控制量作为当前被控量参考值
	sTempAuto->justevaled = true;		// 先设置PID参数可计算
	
	if (!sTempAuto->running)
	{
		// 如果首次运行，初始化所有变量
		sTempAuto->peakType = 0;		//未存在峰值
		sTempAuto->peakCount = 0;		// 峰值数量	
		sTempAuto->justchanged = false;		// 峰值状态改变标志
		sTempAuto->absMax = refVal;
		sTempAuto->absMin = refVal;
		sTempAuto->running = true;		// 将系统设为运行
		//  先将输出设为PWM峰顶
		*(sTempAuto->output) = sTempAuto->outputStart + sTempAuto->oStep;
	}
	else
	{
		// 在采样过程中，不断的找到最大最小值
		if (refVal > sTempAuto->absMax)
		{
			sTempAuto->absMax = refVal;
		}
		if (refVal < sTempAuto->absMin)
		{
			sTempAuto->absMin = refVal;
		}
	}

	// 总结：控制量超过触发线，输出反向，以便输出和控制量永远处于反向状态，这样子，可以维持控制在设定值附近
	if (refVal > *(sTempAuto->setpoint) + sTempAuto->noiseBand) 
	{
		*(sTempAuto->output) = sTempAuto->outputStart - sTempAuto->oStep;
	}
	else if (refVal < *(sTempAuto->setpoint) - sTempAuto->noiseBand) 
	{
		*(sTempAuto->output) = sTempAuto->outputStart + sTempAuto->oStep;
	}

	sTempAuto->isMax = true; sTempAuto->isMin = true;		//初始化最大最小值标志位
	// 回溯数量，按照先进先出的原则将控制量存储在sTempAuto->lastInputs中，最新的值存在0，依次类推
	for (i = sTempAuto->nLookBack - 1; i >= 0; i--)
	{
		double val = sTempAuto->lastInputs[i];
		//如果 参考值前面所有点大，那么是可能的最大值
		if (sTempAuto->isMax) 
		{
			sTempAuto->isMax = refVal > val;
		}
		//如果 参考值比前面所有的点小，那么是可能的最小值
		if (sTempAuto->isMin) 
		{
			sTempAuto->isMin = refVal < val;
		}
		// 更新队列，先进先出
		sTempAuto->lastInputs[i + 1] = sTempAuto->lastInputs[i];
	}
	sTempAuto->lastInputs[0] = refVal;

	//如果sTempAuto->nLookBack小于9，那么计算失败，退出，也就是意味着，窗口必须比9个点大。
	if (sTempAuto->nLookBack < 9)
	{
		// 如果储存的控制量的值数量小于9个，那么，计算结果将不可信，这个地方可以稍作修改
		return 0;
	}

	// 找极值的过程
	if (sTempAuto->isMax)
	{
		// 如果峰值标志位0，那么置1，即可能是峰顶
		if (sTempAuto->peakType == 0)sTempAuto->peakType = 1;
		// 如果是-1，那么置1，设置转换标志位真，sTempAuto->peak2 = sTempAuto->peak1
		if (sTempAuto->peakType == -1)
		{
			sTempAuto->peakType = 1;
			// 状态转换
			sTempAuto->justchanged = true;
			// 记录峰值时间
			sTempAuto->peak2 = sTempAuto->peak1;
		}
		// sTempAuto->peak1 设置为当前运行时间
		sTempAuto->peak1 = now;
		//  可能的峰值 = 当前控制量
		sTempAuto->peaks[sTempAuto->peakCount] = refVal;
	}
	//如果是最小值
	else if (sTempAuto->isMin)
	{
		// 如果峰值标志位0，那么置-1
		if (sTempAuto->peakType == 0)sTempAuto->peakType = -1;
		// 如果是1，那么置-1，设置转换标志位真，sTempAuto->peakCount ++，这个量只有到了峰谷才会增加，但无论峰顶还是峰谷都会更新为可能的极值
		if (sTempAuto->peakType == 1)
		{
			sTempAuto->peakType = -1;
			sTempAuto->peakCount++;
			sTempAuto->justchanged = true;
		}
		// 如果sTempAuto->peakCount 小于10， 那么峰值数组设置为参考值
		if (sTempAuto->peakCount < 10) sTempAuto->peaks[sTempAuto->peakCount] = refVal;
	}

	if (sTempAuto->justchanged && sTempAuto->peakCount > 3)
	{ 
		double avgSeparation = (fabs(sTempAuto->peaks[sTempAuto->peakCount - 1] - sTempAuto->peaks[sTempAuto->peakCount - 2]) + fabs(sTempAuto->peaks[sTempAuto->peakCount - 2] - sTempAuto->peaks[sTempAuto->peakCount - 3])) / 2;
		// 如果平均avg 在峰峰值5%内波动，执行finishup 并结束运行， 返回成功标志位，峰顶稳定了才开始调节
		if (avgSeparation < 0.05*(sTempAuto->absMax - sTempAuto->absMin))
		{
			FinishUp(sTempAuto);
			sTempAuto->running = false;
			return 1;
		}
	}
	
	// 如果没有设置 转换为0 ， 失败退出
	sTempAuto->justchanged = false;
	return 0;
}

void FinishUp(TEMP_AUTO_STRUCT *sTempAuto)
{
	*(sTempAuto->output) = sTempAuto->outputStart;
	sTempAuto->Ku = 4 * (2 * sTempAuto->oStep) / ((sTempAuto->absMax - sTempAuto->absMin)*3.14159);
	sTempAuto->Pu = (double)(sTempAuto->peak1 - sTempAuto->peak2) / 1000;
}

// 此处三个函数用于更改PID参数
double GetKpAuto(TEMP_AUTO_STRUCT *sTempAuto)
{
	return sTempAuto->controlType == 1 ? 0.6 * sTempAuto->Ku : 0.4 * sTempAuto->Ku;
}

double GetKiAuto(TEMP_AUTO_STRUCT *sTempAuto)
{
	return sTempAuto->controlType == 1 ? 1.2*sTempAuto->Ku / sTempAuto->Pu : 0.48 * sTempAuto->Ku / sTempAuto->Pu;  // Ki = Kc/Ti
}

double GetKdAuto(TEMP_AUTO_STRUCT *sTempAuto)
{
	return sTempAuto->controlType == 1 ? 0.075 * sTempAuto->Ku * sTempAuto->Pu : 0;  //Kd = Kc * Td
}

// 此处用于设置输出output 的半峰值
void SetOutputStep(double Step, TEMP_AUTO_STRUCT *sTempAuto)
{
	sTempAuto->oStep = Step;
}


// 用于获取output半峰值
double GetOutputStep(TEMP_AUTO_STRUCT *sTempAuto)
{
	return sTempAuto->oStep;
}

// 设置控制模式 0 代表PI  1代表 PID
void SetControlType(int Type, TEMP_AUTO_STRUCT *sTempAuto) //0=PI, 1=PID
{
	sTempAuto->controlType = Type;
}

//获取当前的控制模式  PID还是PID
int GetControlType(TEMP_AUTO_STRUCT *sTempAuto)
{
	return sTempAuto->controlType;
}

// 设置噪声带宽
void SetNoiseBand(double Band, TEMP_AUTO_STRUCT *sTempAuto)
{
	sTempAuto->noiseBand = Band;
}

//获取噪声带宽
double GetNoiseBand(TEMP_AUTO_STRUCT *sTempAuto)
{
	return sTempAuto->noiseBand;
}

//获取lookback时间，最小为1. 小于25，设定采样值为250，lookback数量为100，否则，数量为100，采样周期在value基础上放大10倍
void SetLookbackSec(int value, TEMP_AUTO_STRUCT *sTempAuto)
{
	if (value < 1) value = 1;

	if (value < 25)
	{
		sTempAuto->nLookBack = value * 4;
		sTempAuto->sampleTime = 250;
	}
	else
	{
		sTempAuto->nLookBack = 100;
		sTempAuto->sampleTime = value * 10;
	}
}

//获取lookback时间，转换为s
int GetLookbackSec(TEMP_AUTO_STRUCT *sTempAuto)
{
	return sTempAuto->nLookBack * sTempAuto->sampleTime / 1000;
}


void SetStartOutput(double StartVal, TEMP_AUTO_STRUCT *sTempAuto)
{
	sTempAuto->outputStart = StartVal;
}

double GettStartOutput(TEMP_AUTO_STRUCT *sTempAuto)
{
	return sTempAuto->outputStart;
}

