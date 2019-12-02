/**********************************************************************************************
*  ˵�������ļ����PID�������ƣ�������������������趨 
*  ���ߣ���ͥ��
*  ���䣺kyzy_duck@163.com
**********************************************************************************************/
#include "PID_v1.h"
// ���캯��
PID::PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki,
		 double Kd, int POn, int ControllerDirection, void *time)
{
	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;
	inAuto = false;
	// Ĭ��0-255������
	PID::SetOutputLimits(0, 255);
	// Ĭ��100ms
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
* ����PID���ֵ�������˺�����Ҫ��ÿ��ѭ����ʱ�����
**********************************************************************************/
bool PID::Compute()
{
	if (!inAuto)
	{
		return false;
	}
	unsigned long now = *currentTime;
	unsigned long timeChange = (now - lastTime);
	// ȷ��ʱ���Ƿ������������
	if (timeChange >= SampleTime)
	{
		double input = *myInput;
		double error = *mySetpoint - input;
		double dInput = (input - lastInput);
		// �����
		outputSum += (ki * error);

		// ����ص���������ô��
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

		// ���ָ���˱������ô���ӱ�����
		double output = 0;
		if (pOnE)
		{
			output = kp * error;
		}
		else
		{
			output = 0;
		}

		// ����PID���
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

		// ������һ�μ������
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
* �˺�����������PID������һ���ڹ���ʱ����Ѿ����ò�����ϣ���PID���й����У�Ҳ���Ե��ô˺����������Ʋ���
******************************************************************************/
void PID::SetTunings(double v_Kp, double v_Ki, double v_Kd, int v_POn)
{
	// ���Ϊ�������˳�
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

	// ���Ϊ������������
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
* ���ò������ڣ���msΪ��λ
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
*  �˺�������������ĵ������Сֵ�������ڲ���PWM
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
* �����趨/��ȡPID����������/�ر�
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
* PID������ʱ����Ҫ���ô˺������Զ����ã������û�����
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
* ���ַ�ʽ�趨/��ȡ���Ʒ����������DIRECT��+���������+�����룬�������REVERSE��+�����������-������
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
* ���ڻ�ȡPID���Ʋ���
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


