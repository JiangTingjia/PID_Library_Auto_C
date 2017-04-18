/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Nor.h
*Note: ���ļ����PID���ڵı�Ҫ���ܣ�����Ҫ�޸�
----------------------------------------------------------------------------------------*/
#include "../include/PID_Nor.h"

/**********************************����***************************/
void PID(double* Input, double* Output, double* Setpoint, unsigned int *iCurTime, TEMP_NOR_STRUCT *sTempNor)
{
	// �趨�������
	double outMin = 0;
	double outMax = 255;
	unsigned int SampleTime = 100;	
	int ControllerDirection = DIRECT;
	double Kp = outMax * 0.01;
	double Ki = 0;
	double Kd = 0;
	int Mode = AUTOMATIC;
	sTempNor->Current_time = iCurTime;
	sTempNor->myOutput = Output;	// ��ʼ��������
	sTempNor->myInput = Input;	//��ʼ����������
	sTempNor->mySetpoint = Setpoint;	//��ʼ���趨ֵ
	sTempNor->inAuto = false;		// �ȹر�PID����
	SetOutputLimits(outMin, outMax, sTempNor);				//Ĭ��PID����Ϊ0��255
	// Ĭ�ϲ�������Ϊ100ms
	SetSampleTime(SampleTime, sTempNor);
	SetControllerDirection(ControllerDirection, sTempNor); // ��������ķ���
	SetTunings(Kp, Ki, Kd, sTempNor);			// ����PID ���Ʋ���
	SetMode(Mode, sTempNor);		// �趨PID���ƿ���
	sTempNor->lastTimeNor = *(sTempNor->Current_time) - sTempNor->SampleTime; 		// ��ȡ��ʼ��PID��Ӧ��ϵͳ����ʱ��
}

bool Compute(TEMP_NOR_STRUCT *sTempNor)
{
	unsigned int now = 0;		// ���ڴ洢������㺯����Ӧ��ϵͳ����ʱ��
	unsigned int timeChange = 0;	// ���ڴ���˴ν�������ʱ��仯
	
	if (!sTempNor->inAuto) 	// ����ر�PID��ֱ���˳�
	{
		return false;	
	}		
	
	now = *(sTempNor->Current_time);		// ���㵽ĿǰΪֹ�����˶���ms
	timeChange = (now - sTempNor->lastTimeNor);
	
	if (timeChange >= sTempNor->SampleTime)		//��������ŵ�ʱ����ڻ��ߵ��ڲ���ʱ�䣬��ô�����, �����������������ֱ���˳�
	{
		double dInput = 0.0;
		double output = 0.0;
		double input = *(sTempNor->myInput);		//���Ʊ���������ʱ����
		double error = *(sTempNor->mySetpoint) - input;	// ƫ����
		sTempNor->ITerm += (sTempNor->kiNor * error);		//���������
		// ȷ�������ޣ����޸�ֵΪ����ֵ
		if (sTempNor->ITerm > sTempNor->outMax) sTempNor->ITerm = sTempNor->outMax;
		else if (sTempNor->ITerm < sTempNor->outMin) sTempNor->ITerm = sTempNor->outMin;
		dInput = (input - sTempNor->lastInput);	//΢���� 
		output = sTempNor->kpNor * error + sTempNor->ITerm - sTempNor->kdNor * dInput;	//PID �������
		// ȷ�������ޣ����޸�ֵΪ����ֵ
		if (output > sTempNor->outMax) output = sTempNor->outMax;
		else if (output < sTempNor->outMin) output = sTempNor->outMin;
		*(sTempNor->myOutput) = output;		//��ȡ���
		sTempNor->lastInput = input;
		sTempNor->lastTimeNor = now;		//���¼���ʱ��
		return true;
	}
	else 
	{
		return false;
	}
}

void SetTunings(double Kp, double Ki, double Kd, TEMP_NOR_STRUCT *sTempNor)
{
	double SampleTimeInSec = 0.0;			// ������ʱ��ת��Ϊs
	
	if (Kp < 0 || Ki < 0 || Kd < 0)  // PID���ڲ�������Ϊ��
	{
		return;
	}
	
	sTempNor->dispKp = Kp; sTempNor->dispKi = Ki; sTempNor->dispKd = Kd; 		// ������ʾPID����
	SampleTimeInSec = ((double)sTempNor->SampleTime) / 1000;
	// ����PID�����������Ϊ�������ı����ʱ������ͻ�䣬����ԭ��ע��˵�����
	sTempNor->kpNor = Kp;
	sTempNor->kiNor = Ki * SampleTimeInSec;
	sTempNor->kdNor = Kd / SampleTimeInSec;

	//	�趨����
	if (sTempNor->controllerDirection == REVERSE)
	{
		sTempNor->kpNor = (0 - sTempNor->kpNor);
		sTempNor->kiNor = (0 - sTempNor->kiNor);
		sTempNor->kdNor = (0 - sTempNor->kdNor);
	}
}

void SetSampleTime(unsigned int NewSampleTime, TEMP_NOR_STRUCT *sTempNor)
{
	// �ı����ʱ��
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
	
	if (newAuto == !sTempNor->inAuto)	// ���ģʽ��һ������ô�����³�ʼ��
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
	// ����رգ���رգ��������������
	return  sTempNor->inAuto ? AUTOMATIC : MANUAL;
}
int GetDirection(TEMP_NOR_STRUCT *sTempNor)
{ 
	return sTempNor->controllerDirection;
}

//**************ENDFILE****************//
