/**********************************************************************************************
*  说明：此文件完成PID基本控制，包含控制器所有相关设定 
*  作者：蒋庭佳
*  邮箱：kyzy_duck@163.com
**********************************************************************************************/
#include "PID_v1.h"
// 构造函数
PID::PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki,
		 double Kd, int POn, int ControllerDirection, void *time)
{
	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;
	inAuto = false;
	// 默认0-255的限制
	PID::SetOutputLimits(0, 255);
	// 默认100ms
	PID::SetSampleTime(100);
	PID::SetDirection(ControllerDirection);
	PID::SetTunings(Kp, Ki, Kd, POn);
	currentTime = (unsigned long *)time;
	lastTime = *currentTime - SampleTime;
}

PID::PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki,
		 double Kd, int ControllerDirection, void *time)
	: PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection, time) {}

/***********************************************************************
* 计算PID输出值函数，此函数需要在每次循环的时候进行
**********************************************************************************/
bool PID::Compute()
{
	if (!inAuto)
	{
		return false;
	}
	unsigned long now = *currentTime;
	unsigned long timeChange = (now - lastTime);
	// 确认时间是否满足计算周期
	if (timeChange >= SampleTime)
	{
		double input = *myInput;
		double error = *mySetpoint - input;
		double dInput = (input - lastInput);
		// 求积分
		outputSum += (ki * error);

		// 如果关掉比例向，那么就
		if (!pOnE)
		{
			outputSum -= kp * dInput;
		}

		if (outputSum > outMax)
		{
			outputSum = outMax;
		}
		else if (outputSum < outMin)
		{
			outputSum = outMin;
		}

		// 如果指定了比例项，那么增加比例项
		double output = 0;
		if (pOnE)
		{
			output = kp * error;
		}
		else
		{
			output = 0;
		}

		// 计算PID输出
		output += outputSum - kd * dInput;

		if (output > outMax)
		{
			output = outMax;
		}
		else if (output < outMin)
		{
			output = outMin;
		}
		*myOutput = output;

		// 更新下一次计算参数
		lastInput = input;
		lastTime = now;
		return true;
	}
	else
	{
		return false;
	}
}

/**************************************************************
* 此函数用于设置PID参数，一般在构造时候就已经设置参数完毕，在PID运行过程中，也可以调用此函数调整控制参数
******************************************************************************/
void PID::SetTunings(double v_Kp, double v_Ki, double v_Kd, int v_POn)
{
	// 如果为负，就退出
	if (v_Kp < 0 || v_Ki < 0 || v_Kd < 0)
	{
		return;
	}

	pOn = v_POn;
	pOnE = (v_POn == P_ON_E);

	dispKp = v_Kp;
	dispKi = v_Ki;
	dispKd = v_Kd;

	double SampleTimeInSec = ((double)SampleTime) / 1000;
	kp = v_Kp;
	ki = v_Ki * SampleTimeInSec;
	kd = v_Kd / SampleTimeInSec;

	// 如果为负，参数反向
	if (controllerDirection == REVERSE)
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
}

void PID::SetTunings(double v_Kp, double v_Ki, double v_Kd)
{
	SetTunings(v_Kp, v_Ki, v_Kd, pOn);
}

/**********************************************************
* 设置采样周期，以ms为单位
******************************************************************************/
void PID::SetSampleTime(unsigned int NewSampleTime)
{
	if (NewSampleTime > 0)
	{
		double ratio = (double)NewSampleTime / (double)SampleTime;
		ki *= ratio;
		kd /= ratio;
		SampleTime = (unsigned int)NewSampleTime;
	}
}
unsigned int PID::GetSampleTime(void)
{
	return SampleTime;
}

/*****************************************************
*  此函数用于设置输的的最大最小值，可用于产生PWM
**************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
	if (Min >= Max)
	{
		return;
	}
	outMin = Min;
	outMax = Max;

	if (inAuto)
	{
		if (*myOutput > outMax)
		{
			*myOutput = outMax;
		}
		else if (*myOutput < outMin)
		{
			*myOutput = outMin;
		}

		if (outputSum > outMax)
		{
			outputSum = outMax;
		}
		else if (outputSum < outMin)
		{
			outputSum = outMin;
		}
	}
}
double PID::GetOutputMin(void)
{
	return outMin;
}
double PID::GetOutputMax(void)
{
	return outMax;
}

/*****************************************************************
* 用于设定/获取PID控制器开启/关闭
******************************************************************************/
void PID::SetMode(int Mode)
{
	bool newAuto = (Mode == AUTOMATIC);
	if (newAuto && !inAuto)
	{
		PID::Initialize();
	}
	inAuto = newAuto;
}
int PID::GetMode()
{
	return inAuto ? AUTOMATIC : MANUAL;
}

/*****************************************************************
* PID开启的时候需要调用此函数，自动调用，无需用户调用
******************************************************************************/
void PID::Initialize()
{
	outputSum = *myOutput;
	lastInput = *myInput;
	if (outputSum > outMax)
	{
		outputSum = outMax;
	}
	else if (outputSum < outMin)
	{
		outputSum = outMin;
	}
}

/*************************************************
* 两种方式设定/获取控制方向，正向控制DIRECT，+的输出导致+的输入，反向控制REVERSE，+正的输出导致-的输入
******************************************************************************/
void PID::SetDirection(int Direction)
{
	if (inAuto && Direction != controllerDirection)
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
	controllerDirection = Direction;
}
int PID::GetDirection()
{
	return controllerDirection;
}

/**************************************************************
* 用于获取PID控制参数
******************************************************************************/
double PID::GetKp()
{
	return dispKp;
}
double PID::GetKi()
{
	return dispKi;
}
double PID::GetKd()
{
	return dispKd;
}
int PID::GetPonE()
{
	return int(pOnE);
}


