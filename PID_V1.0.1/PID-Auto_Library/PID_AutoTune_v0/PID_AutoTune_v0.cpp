/**********************************************************************************************
*  说明：此文件完成PID自整定
*  作者：蒋庭佳
*  邮箱：kyzy_duck@163.com
*  编码：GBK2312
**********************************************************************************************/
#include "PID_AutoTune_v0.h"
#include "math.h"

PID_ATune::PID_ATune(double *Input, double *Output, void *time)
{
	input = Input;
	output = Output;
	// 自整定模式， 0：PI控制， 1：PID控制
	controlType = 0; 
	noiseBand = 0.5;
	running = false;
	oStep = 30;
	SetLookbackSec(10);
	currentTime = (unsigned long *)time;
	lastTime = *currentTime;
}

void PID_ATune::Cancel()
{
	running = false;
}

int PID_ATune::Runtime()
{
	justevaled = false;
	if (peakCount > 9 && running)
	{
		running = false;
		FinishUp();
		return 1;
	}
	unsigned long now = *currentTime;

	if ((now - lastTime) < sampleTime)
	{
		return false;
	}

	lastTime = now;
	double refVal = *input;
	justevaled = true;
	
	if (!running)
	{
		peakType = 0;
		peakCount = 0;
		justchanged = false;
		absMax = refVal;
		absMin = refVal;
		setpoint = refVal;
		running = true;
		outputStart = *output;
		*output = outputStart + oStep;
	}
	else
	{
		if (refVal > absMax)
		{
			absMax = refVal;
		}
		if (refVal < absMin)
		{
			absMin = refVal;
		}
	}
	// 这里没分方向
	if (refVal > setpoint + noiseBand)
	{
		*output = outputStart - oStep;
	}
	else if (refVal < setpoint - noiseBand)
	{
		*output = outputStart + oStep;
	}

	isMax = true;
	isMin = true;
	for (int i = nLookBack - 1; i >= 0; i--)
	{
		double val = lastInputs[i];
		if (isMax)
		{
			isMax = refVal > val;
		}
		if (isMin)
		{
			isMin = refVal < val;
		}
		lastInputs[i + 1] = lastInputs[i];
	}

	lastInputs[0] = refVal;

	if (nLookBack < 9)
	{
		return 0;
	}

	if (isMax)
	{
		if (peakType == 0)
		{
			peakType = 1;
		}
		if (peakType == -1)
		{
			peakType = 1;
			justchanged = true;
			peak2 = peak1;
		}
		peak1 = now;
		peaks[peakCount] = refVal;
	}
	else if (isMin)
	{
		if (peakType == 0)
		{
			peakType = -1;
		}
		if (peakType == 1)
		{
			peakType = -1;
			peakCount++;
			justchanged = true;
		}

		if (peakCount < 10)
			peaks[peakCount] = refVal;
	}

	if (justchanged && peakCount > 2)
	{
		double avgSeparation = (fabs(peaks[peakCount - 1] - peaks[peakCount - 2]) +
								fabs(peaks[peakCount - 2] - peaks[peakCount - 3])) /
							   2;
		if (avgSeparation < 0.05 * (absMax - absMin))
		{
			FinishUp();
			running = false;
			return 1;
		}
	}
	justchanged = false;
	return 0;
}
void PID_ATune::FinishUp()
{
	*output = outputStart;
	Ku = 4 * (2 * oStep) / ((absMax - absMin) * 3.14159);
	Pu = (double)(peak1 - peak2) / 1000;
}

double PID_ATune::GetKp()
{
	return controlType == 1 ? 0.6 * Ku : 0.4 * Ku;
}

double PID_ATune::GetKi()
{
	return controlType == 1 ? 1.2 * Ku / Pu : 0.48 * Ku / Pu; // Ki = Kc/Ti
}

double PID_ATune::GetKd()
{
	return controlType == 1 ? 0.075 * Ku * Pu : 0; // Kd = Kc * Td
}

void PID_ATune::SetOutputStep(double Step)
{
	oStep = Step;
}

double PID_ATune::GetOutputStep()
{
	return oStep;
}

void PID_ATune::SetControlType(int Type) // 0=PI, 1=PID
{
	controlType = Type;
}
int PID_ATune::GetControlType()
{
	return controlType;
}

void PID_ATune::SetNoiseBand(double Band)
{
	noiseBand = Band;
}

double PID_ATune::GetNoiseBand()
{
	return noiseBand;
}

void PID_ATune::SetLookbackSec(int value)
{
	if (value < 1)
	{
		value = 1;
	}

	if (value < 25)
	{
		nLookBack = value * 4;
		sampleTime = 250;
	}
	else
	{
		nLookBack = 100;
		sampleTime = value * 10;
	}
}

int PID_ATune::GetLookbackSec()
{
	return nLookBack * sampleTime / 1000;
}
