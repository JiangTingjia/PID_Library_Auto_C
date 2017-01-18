/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Auto.c
*Note: ���ļ���� PID �Զ����α�Ҫ���ܣ�����Ҫ�޸�
----------------------------------------------------------------------------------------*/
#include "PID_Auto.h"
#include "math.h"

/**********************************����***************************/
void PID_ATune(double* Input, double* Output, double *setpoint, unsigned int *iCurTime, TEMP_AUTO_STRUCT *sTempAuto)
{
	int Type; 	//Ĭ��ΪPID����
	double noiseBand;	// Ĭ����������
	double oStep;		// Ĭ��PWM ���ֵ
	int LookBack;		// �趨��������
	double StartVal;	// �趨PWM�����׼ֵ
	sTempAuto->Current_time = iCurTime;
	// ��ֵ������ ������
	sTempAuto->input = Input;
	sTempAuto->output = Output;
	sTempAuto->setpoint = setpoint;
	Type = 0;				//Ĭ��PI����
	SetControlType(Type, sTempAuto);
	noiseBand = *(sTempAuto->setpoint) * 0.02;	//Ĭ������Ϊ�趨ֵ��2%
	SetNoiseBand(noiseBand, sTempAuto);
	oStep = *(sTempAuto->output);			//Ĭ��PWM���Ϊ��ǰ���
	SetOutputStep(oStep, sTempAuto);
	StartVal = *(sTempAuto->output);		//Ĭ��PWM��׼ֵΪ��ǰ���
	SetStartOutput(StartVal, sTempAuto);
	LookBack = 10;							//Ĭ�ϻ�������Ϊ10
	SetLookbackSec(LookBack, sTempAuto);	
	sTempAuto->running = false;	// Ĭ���Զ���гΪ�ر�
	sTempAuto->lastTimeAuto = *(sTempAuto->Current_time);
}

//	�ر� �Զ���г
void Cancel(TEMP_AUTO_STRUCT *sTempAuto)
{
	sTempAuto->running = false;
}

//	�����г���������Ƴ���
// ����Զ���г�޷��˳������Ը���start �� ostep����������
int Runtime(TEMP_AUTO_STRUCT *sTempAuto)
{
	unsigned int now = 0;		// ���ڴ洢 ��ǰϵͳ����ʱ��
	double refVal = 0.0;		//��¼�ս��뺯����Ӧ�ı�����
	int i = 0;					//����forѭ���ı���
	sTempAuto->justevaled = false;				// ��ʾĿǰ���ܼ���PID����
	// ����Ѿ������ķ嶥����������ֵ�������Զ���г��������ô����PID����
	if (sTempAuto->peakCount > 5 && sTempAuto->running)
	{
		sTempAuto->running = false;	//���Զ���г�ر�
		FinishUp(sTempAuto);		// �����������������PID������г���ӵ�
		return 1;
	}
	now = *(sTempAuto->Current_time);		// ��ȡ��ǰʱ�䣬��ʱ����Ҫ�������ж�ʱ�����Ƿ񳬹��������
	/***********************************************/
	//�����ǰ����ʱ�����ʼ����ʱ���С�ڲ���ʱ�䣬��ô�����˳�������ʧ�ܣ�Ҳ������ζ�ţ�δ������������������
	if ((now - sTempAuto->lastTimeAuto) < sTempAuto->sampleTime) 
	{
		return 0;
	}
	/***********************************************/
	sTempAuto->lastTimeAuto = now;		//��ȡ�˴μ����Ӧ��ϵͳ����ʱ��
	refVal = *(sTempAuto->input);		//��ȡ��ʱ�ı���������Ϊ��ǰ�������ο�ֵ
	sTempAuto->justevaled = true;		// ������PID�����ɼ���
	
	if (!sTempAuto->running)
	{
		// ����״����У���ʼ�����б���
		sTempAuto->peakType = 0;		//δ���ڷ�ֵ
		sTempAuto->peakCount = 0;		// ��ֵ����	
		sTempAuto->justchanged = false;		// ��ֵ״̬�ı��־
		sTempAuto->absMax = refVal;
		sTempAuto->absMin = refVal;
		sTempAuto->running = true;		// ��ϵͳ��Ϊ����
		//  �Ƚ������ΪPWM�嶥
		*(sTempAuto->output) = sTempAuto->outputStart + sTempAuto->oStep;
	}
	else
	{
		// �ڲ��������У����ϵ��ҵ������Сֵ
		if (refVal > sTempAuto->absMax)
		{
			sTempAuto->absMax = refVal;
		}
		if (refVal < sTempAuto->absMin)
		{
			sTempAuto->absMin = refVal;
		}
	}

	// �ܽ᣺���������������ߣ���������Ա�����Ϳ�������Զ���ڷ���״̬�������ӣ�����ά�ֿ������趨ֵ����
	if (refVal > *(sTempAuto->setpoint) + sTempAuto->noiseBand) 
	{
		*(sTempAuto->output) = sTempAuto->outputStart - sTempAuto->oStep;
	}
	else if (refVal < *(sTempAuto->setpoint) - sTempAuto->noiseBand) 
	{
		*(sTempAuto->output) = sTempAuto->outputStart + sTempAuto->oStep;
	}

	sTempAuto->isMax = true; sTempAuto->isMin = true;		//��ʼ�������Сֵ��־λ
	// ���������������Ƚ��ȳ���ԭ�򽫿������洢��sTempAuto->lastInputs�У����µ�ֵ����0����������
	for (i = sTempAuto->nLookBack - 1; i >= 0; i--)
	{
		double val = sTempAuto->lastInputs[i];
		//��� �ο�ֵǰ�����е����ô�ǿ��ܵ����ֵ
		if (sTempAuto->isMax) 
		{
			sTempAuto->isMax = refVal > val;
		}
		//��� �ο�ֵ��ǰ�����еĵ�С����ô�ǿ��ܵ���Сֵ
		if (sTempAuto->isMin) 
		{
			sTempAuto->isMin = refVal < val;
		}
		// ���¶��У��Ƚ��ȳ�
		sTempAuto->lastInputs[i + 1] = sTempAuto->lastInputs[i];
	}
	sTempAuto->lastInputs[0] = refVal;

	//���sTempAuto->nLookBackС��9����ô����ʧ�ܣ��˳���Ҳ������ζ�ţ����ڱ����9�����
	if (sTempAuto->nLookBack < 9)
	{
		// �������Ŀ�������ֵ����С��9������ô���������������ţ�����ط����������޸�
		return 0;
	}

	// �Ҽ�ֵ�Ĺ���
	if (sTempAuto->isMax)
	{
		// �����ֵ��־λ0����ô��1���������Ƿ嶥
		if (sTempAuto->peakType == 0)sTempAuto->peakType = 1;
		// �����-1����ô��1������ת����־λ�棬sTempAuto->peak2 = sTempAuto->peak1
		if (sTempAuto->peakType == -1)
		{
			sTempAuto->peakType = 1;
			// ״̬ת��
			sTempAuto->justchanged = true;
			// ��¼��ֵʱ��
			sTempAuto->peak2 = sTempAuto->peak1;
		}
		// sTempAuto->peak1 ����Ϊ��ǰ����ʱ��
		sTempAuto->peak1 = now;
		//  ���ܵķ�ֵ = ��ǰ������
		sTempAuto->peaks[sTempAuto->peakCount] = refVal;
	}
	//�������Сֵ
	else if (sTempAuto->isMin)
	{
		// �����ֵ��־λ0����ô��-1
		if (sTempAuto->peakType == 0)sTempAuto->peakType = -1;
		// �����1����ô��-1������ת����־λ�棬sTempAuto->peakCount ++�������ֻ�е��˷�ȲŻ����ӣ������۷嶥���Ƿ�ȶ������Ϊ���ܵļ�ֵ
		if (sTempAuto->peakType == 1)
		{
			sTempAuto->peakType = -1;
			sTempAuto->peakCount++;
			sTempAuto->justchanged = true;
		}
		// ���sTempAuto->peakCount С��10�� ��ô��ֵ��������Ϊ�ο�ֵ
		if (sTempAuto->peakCount < 10) sTempAuto->peaks[sTempAuto->peakCount] = refVal;
	}

	if (sTempAuto->justchanged && sTempAuto->peakCount > 3)
	{ 
		double avgSeparation = (fabs(sTempAuto->peaks[sTempAuto->peakCount - 1] - sTempAuto->peaks[sTempAuto->peakCount - 2]) + fabs(sTempAuto->peaks[sTempAuto->peakCount - 2] - sTempAuto->peaks[sTempAuto->peakCount - 3])) / 2;
		// ���ƽ��avg �ڷ��ֵ5%�ڲ�����ִ��finishup ���������У� ���سɹ���־λ���嶥�ȶ��˲ſ�ʼ����
		if (avgSeparation < 0.05*(sTempAuto->absMax - sTempAuto->absMin))
		{
			FinishUp(sTempAuto);
			sTempAuto->running = false;
			return 1;
		}
	}
	
	// ���û������ ת��Ϊ0 �� ʧ���˳�
	sTempAuto->justchanged = false;
	return 0;
}

void FinishUp(TEMP_AUTO_STRUCT *sTempAuto)
{
	*(sTempAuto->output) = sTempAuto->outputStart;
	sTempAuto->Ku = 4 * (2 * sTempAuto->oStep) / ((sTempAuto->absMax - sTempAuto->absMin)*3.14159);
	sTempAuto->Pu = (double)(sTempAuto->peak1 - sTempAuto->peak2) / 1000;
}

// �˴������������ڸ���PID����
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

// �˴������������output �İ��ֵ
void SetOutputStep(double Step, TEMP_AUTO_STRUCT *sTempAuto)
{
	sTempAuto->oStep = Step;
}


// ���ڻ�ȡoutput���ֵ
double GetOutputStep(TEMP_AUTO_STRUCT *sTempAuto)
{
	return sTempAuto->oStep;
}

// ���ÿ���ģʽ 0 ����PI  1���� PID
void SetControlType(int Type, TEMP_AUTO_STRUCT *sTempAuto) //0=PI, 1=PID
{
	sTempAuto->controlType = Type;
}

//��ȡ��ǰ�Ŀ���ģʽ  PID����PID
int GetControlType(TEMP_AUTO_STRUCT *sTempAuto)
{
	return sTempAuto->controlType;
}

// ������������
void SetNoiseBand(double Band, TEMP_AUTO_STRUCT *sTempAuto)
{
	sTempAuto->noiseBand = Band;
}

//��ȡ��������
double GetNoiseBand(TEMP_AUTO_STRUCT *sTempAuto)
{
	return sTempAuto->noiseBand;
}

//��ȡlookbackʱ�䣬��СΪ1. С��25���趨����ֵΪ250��lookback����Ϊ100����������Ϊ100������������value�����ϷŴ�10��
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

//��ȡlookbackʱ�䣬ת��Ϊs
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

