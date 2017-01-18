/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Auto.c
*Note: This file is used for PID Autoning
----------------------------------------------------------------------------------------*/
#include "PID_Auto.h"
#include "math.h"

/**********************************º¯Êý***************************/
void PID_ATune(double* Input, double* Output, double *setpoint, unsigned int *iCurTime, TEMP_AUTO_STRUCT *sTempAuto)
{
	int Type; 	
	double noiseBand;	
	double oStep;		
	int LookBack;		
	double StartVal;	
	sTempAuto->Current_time = iCurTime;
	// connect pointer
	sTempAuto->input = Input;
	sTempAuto->output = Output;
	sTempAuto->setpoint = setpoint;
	Type = 0;				// default PI
	SetControlType(Type, sTempAuto);
	noiseBand = *(sTempAuto->setpoint) * 0.02;	//default  2%SP
	SetNoiseBand(noiseBand, sTempAuto);
	oStep = *(sTempAuto->output);			// default CV
	SetOutputStep(oStep, sTempAuto);
	StartVal = *(sTempAuto->output);		// default CV
	SetStartOutput(StartVal, sTempAuto);
	LookBack = 10;							// default 10
	SetLookbackSec(LookBack, sTempAuto);	
	sTempAuto->running = false;	
	sTempAuto->lastTimeAuto = *(sTempAuto->Current_time);
}

void Cancel(TEMP_AUTO_STRUCT *sTempAuto)
{
	sTempAuto->running = false;
}

//	The AUTO process
int Runtime(TEMP_AUTO_STRUCT *sTempAuto)
{
	unsigned int now = 0;		
	double refVal = 0.0;		
	int i = 0;					
	sTempAuto->justevaled = false;				
	if (sTempAuto->peakCount > 5 && sTempAuto->running)
	{
		sTempAuto->running = false;	
		FinishUp(sTempAuto);		
		return 1;
	}
	now = *(sTempAuto->Current_time);		
	/***********************************************/
	if ((now - sTempAuto->lastTimeAuto) < sTempAuto->sampleTime) 
	{
		return 0;
	}
	/***********************************************/
	sTempAuto->lastTimeAuto = now;		
	refVal = *(sTempAuto->input);		
	sTempAuto->justevaled = true;		
	
	if (!sTempAuto->running)
	{
		//initialize working variables the first time around
		sTempAuto->peakType = 0;		
		sTempAuto->peakCount = 0;		
		sTempAuto->justchanged = false;		
		sTempAuto->absMax = refVal;
		sTempAuto->absMin = refVal;
		sTempAuto->running = true;		
		*(sTempAuto->output) = sTempAuto->outputStart + sTempAuto->oStep;
	}
	else
	{
		if (refVal > sTempAuto->absMax)
		{
			sTempAuto->absMax = refVal;
		}
		if (refVal < sTempAuto->absMin)
		{
			sTempAuto->absMin = refVal;
		}
	}

	//oscillate the output base on the input's relation to the setpoint
	if (refVal > *(sTempAuto->setpoint) + sTempAuto->noiseBand) 
	{
		*(sTempAuto->output) = sTempAuto->outputStart - sTempAuto->oStep;
	}
	else if (refVal < *(sTempAuto->setpoint) - sTempAuto->noiseBand) 
	{
		*(sTempAuto->output) = sTempAuto->outputStart + sTempAuto->oStep;
	}

	sTempAuto->isMax = true; sTempAuto->isMin = true;		
	for (i = sTempAuto->nLookBack - 1; i >= 0; i--)
	{
		double val = sTempAuto->lastInputs[i];
		if (sTempAuto->isMax) 
		{
			sTempAuto->isMax = refVal > val;
		}
		if (sTempAuto->isMin) 
		{
			sTempAuto->isMin = refVal < val;
		}
		sTempAuto->lastInputs[i + 1] = sTempAuto->lastInputs[i];
	}
	sTempAuto->lastInputs[0] = refVal;

	//we don't want to trust the maxes or mins until the inputs array has been filled
	if (sTempAuto->nLookBack < 9)
	{
		return 0;
	}

	if (sTempAuto->isMax)
	{
		if (sTempAuto->peakType == 0)sTempAuto->peakType = 1;
		if (sTempAuto->peakType == -1)
		{
			sTempAuto->peakType = 1;
			sTempAuto->justchanged = true;
			sTempAuto->peak2 = sTempAuto->peak1;
		}
		sTempAuto->peak1 = now;
		sTempAuto->peaks[sTempAuto->peakCount] = refVal;
	}
	else if (sTempAuto->isMin)
	{
		if (sTempAuto->peakType == 0)sTempAuto->peakType = -1;
		if (sTempAuto->peakType == 1)
		{
			sTempAuto->peakType = -1;
			sTempAuto->peakCount++;
			sTempAuto->justchanged = true;
		}
		if (sTempAuto->peakCount < 10) sTempAuto->peaks[sTempAuto->peakCount] = refVal;
	}

	if (sTempAuto->justchanged && sTempAuto->peakCount > 3)
	{ 
		double avgSeparation = (fabs(sTempAuto->peaks[sTempAuto->peakCount - 1] - sTempAuto->peaks[sTempAuto->peakCount - 2]) + fabs(sTempAuto->peaks[sTempAuto->peakCount - 2] - sTempAuto->peaks[sTempAuto->peakCount - 3])) / 2;
		if (avgSeparation < 0.05*(sTempAuto->absMax - sTempAuto->absMin))
		{
			FinishUp(sTempAuto);
			sTempAuto->running = false;
			return 1;
		}
	}
	
	sTempAuto->justchanged = false;
	return 0;
}

void FinishUp(TEMP_AUTO_STRUCT *sTempAuto)
{
	*(sTempAuto->output) = sTempAuto->outputStart;
	sTempAuto->Ku = 4 * (2 * sTempAuto->oStep) / ((sTempAuto->absMax - sTempAuto->absMin)*3.14159);
	sTempAuto->Pu = (double)(sTempAuto->peak1 - sTempAuto->peak2) / 1000;
}

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

void SetOutputStep(double Step, TEMP_AUTO_STRUCT *sTempAuto)
{
	sTempAuto->oStep = Step;
}

double GetOutputStep(TEMP_AUTO_STRUCT *sTempAuto)
{
	return sTempAuto->oStep;
}

void SetControlType(int Type, TEMP_AUTO_STRUCT *sTempAuto) //0=PI, 1=PID
{
	sTempAuto->controlType = Type;
}

int GetControlType(TEMP_AUTO_STRUCT *sTempAuto)
{
	return sTempAuto->controlType;
}

void SetNoiseBand(double Band, TEMP_AUTO_STRUCT *sTempAuto)
{
	sTempAuto->noiseBand = Band;
}

double GetNoiseBand(TEMP_AUTO_STRUCT *sTempAuto)
{
	return sTempAuto->noiseBand;
}

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

