#include "include\PID_Nor.h"
#include "include\PID_Auto.h"
#include <stdbool.h>

// ���������Զ�����ǰNOR�Ĺ���ģʽ �� PI  ���� PID
unsigned int ATuneModeRemember = 2;
// ����
double input = 80, output = 50, setpoint = 180;
double kp = 2, ki = 0.5, kd = 2;
// �Զ����β���
double aTuneStep = 50, aTuneNoise = 1, aTuneStartValue = 100;
unsigned int aTuneLookBack = 20;
// �Զ������Ƿ����ı�־λ
bool tuning = false;

TEMP_NOR_STRUCT sTempNor;
TEMP_AUTO_STRUCT sTempAuto;

// ��iCurTime ���ӵ�ϵͳ������ʱ��
unsigned int iCurTime = 0.0;

// �ָ�����ģʽ
void AutoTuneHelper(bool start)
{
	if (start)
		ATuneModeRemember = GetMode(&sTempNor);
	else
		SetMode(ATuneModeRemember, &sTempNor);
}

// ת���Զ�����״̬
void changeAutoTune()
{
	if (!tuning)
	{
		//Set the output to the desired starting frequency.
		output = aTuneStartValue;
		SetNoiseBand(aTuneNoise, &sTempAuto);
		SetOutputStep(aTuneStep, &sTempAuto);
		SetLookbackSec((int)aTuneLookBack, &sTempAuto);
		AutoTuneHelper(true);
		tuning = true;
	}
	else
	{ //cancel autotune
		Cancel(&sTempAuto);
		tuning = false;
		AutoTuneHelper(false);
	}
}



// ��ʼ��
void setup()
{
	PID(&input, &output, &setpoint, &iCurTime, &sTempNor);		//��ʼ��PID���������������
	PID_ATune(&input, &output, &setpoint, &iCurTime, &sTempAuto);

	//Setup the pid 
	SetMode(AUTOMATIC, &sTempNor);

	if (tuning)
	{
		tuning = false;
		changeAutoTune();
		tuning = true;
	}
}

// ������
void mian()
{
	while (1)
	{
		if (tuning)
		{
			unsigned int val = (Runtime(&sTempAuto));
			if (val != 0)
			{
				tuning = false;
			}
			if (!tuning)
			{ 
				kp = GetKpNor(&sTempNor);
				ki = GetKiNor(&sTempNor);
				kd = GetKdNor(&sTempNor);
				SetTunings(kp, ki, kd, &sTempNor);
				AutoTuneHelper(false);
			}
		}
		else
		{
			Compute(&sTempNor);
		}

	}
}