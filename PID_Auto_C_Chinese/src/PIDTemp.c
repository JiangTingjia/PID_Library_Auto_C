/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PIDTemp.c
*Note: ���ļ����PID���ڵĿ����������
----------------------------------------------------------------------------------------*/
#include "../include/PIDTemp.h"
#include "math.h"

// �ı�PID����
void setupParm(TEMP_PID_STRUCT *sTemp, PID_INIT_PARM sParm)
{
	//��ͨPID����
	SetControllerDirection(sParm.iDirection, &(sTemp->sTempNor));		// �趨���ڷ���
	SetOutputLimits(sParm.dMin, sParm.dMax, &(sTemp->sTempNor));		// �趨�������ķ�Χ
	SetSampleTime(sParm.iSampleTimeNor, &(sTemp->sTempNor));		//�趨����ʱ��
	SetTunings(sParm.KP, sParm.KI, sParm.KD, &(sTemp->sTempNor));			// �趨��г����
	SetMode(sParm.iMode, &(sTemp->sTempNor));		// �趨PID���ƿ���
	// �Զ���г����
	SetControlType(sParm.iType, &(sTemp->sTempAuto));			// �趨PI���ƻ���PID����
	SetNoiseBand(sParm.aTuneNoise, &(sTemp->sTempAuto));		//�趨������ֵ
	SetOutputStep(sParm.aTuneStep, &(sTemp->sTempAuto));		//�趨PWM���ֵ
	SetLookbackSec((int)(sParm.aTuneLookBack), &(sTemp->sTempAuto));	//�趨��������
	// �趨�����ȶ�ʱ��
	sTemp->iSinTimes = sParm.iSinTimes;
	sTemp->iOKTimes = sParm.iOKTimes;
}

// ��ʼ��PID������,��ʼ������
void setupPID(TEMP_PID_STRUCT *sTemp, double *iCurVal, double *iSetVal, double *iOutput,
	unsigned int *iCurTime)
{
	sTemp->inputRun = iCurVal;
	sTemp->setpointRun = iSetVal;
	sTemp->outputRun = iOutput;
	// ��ʼ����ͨPID
	PID(sTemp->inputRun, sTemp->outputRun, sTemp->setpointRun, iCurTime, &(sTemp->sTempNor));		//��ʼ��PID���������������
	PID_ATune(sTemp->inputRun, sTemp->outputRun, sTemp->setpointRun, iCurTime, &(sTemp->sTempAuto));
}

//�����Զ���г�����󣬻ָ���ͨPID����״̬
void AutoTuneHelper(int start, TEMP_PID_STRUCT *sTemp)
{
	if (start)
	{
		// ����ԭ����״̬
		sTemp->ATuneModeRemember = GetMode(&(sTemp->sTempNor));
	}
	else
	{
		// �趨����״̬
		SetMode(sTemp->ATuneModeRemember, &(sTemp->sTempNor));
	}
}

// ����ͨPIDת��Ϊ�Զ���г
void changeAutoTune(TEMP_PID_STRUCT *sTemp)
{
	if (!sTemp->tuning)
	{
		double StartVal = sTemp->iSumOutput / sTemp->iTotalTimes;
		SetStartOutput(StartVal, &(sTemp->sTempAuto));
		SetOutputStep(sTemp->sTempAuto.outputStart, &(sTemp->sTempAuto));
		//  ���俪�� �Զ���г PID����ǰ  PID�������Ĺ���״̬
		AutoTuneHelper(true, sTemp);
		sTemp->tuning = true;
	}
	else
	{ 
		// ȡ���Զ���г �����ظ�ԭPID����������״̬
		Cancel(&(sTemp->sTempAuto));
		sTemp->tuning = false;
		AutoTuneHelper(false, sTemp);
	}
}

// �ж��Ƿ������
bool SinTune(TEMP_PID_STRUCT *sTemp)
{
	// �����гʱ��Ϊ��0����ô�Ͳ������Զ�����
	if (sTemp->iSinTimes <= 0)
	{
		return false;
	}
	//���¶��״δﵽ�趨ֵ��5���Ӻ�ʼ�Զ���г
	double DiffInput = fabs(*(sTemp->inputRun) - *(sTemp->setpointRun));
	sTemp->iAutoTimes = *(sTemp->sTempNor.Current_time);
	if (false == sTemp->iMark && DiffInput <=  sTemp->sTempAuto.noiseBand && *(sTemp->sTempNor.Current_time) > (sTemp->iSinTimes) / 3)
	{
		sTemp->iLastTimes = *(sTemp->sTempNor.Current_time);
		sTemp->iMark = true;
	}
	if (true == sTemp->iMark)
	{
		sTemp->iTotalTimes = sTemp->iTotalTimes + 1;
		sTemp->iSumOutput = sTemp->iSumOutput + *(sTemp->outputRun);
		if (sTemp->iAutoTimes - sTemp->iLastTimes > sTemp->iSinTimes && DiffInput <=  sTemp->sTempAuto.noiseBand)
		{
			sTemp->iMark = false;
			return true;
		}
	}
	return false;
}

// �ж�PID�����Ƿ�����ȶ�
bool PID_OK(TEMP_PID_STRUCT *sTemp)
{
	double DiffInput = 0.0;
	// ���iOKTimesΪ�㣬��ô���ж��Ƿ�����������ֵ��������Ϊ���ڶ�����
	if (0 == sTemp->iOKTimes)
	{
		return true;
	}
	
	if (true == sTemp->iTuneMark)
	{
		DiffInput = fabs(*(sTemp->inputRun) - *(sTemp->setpointRun));
		sTemp->iAutoTimes = *(sTemp->sTempNor.Current_time);
		// ����n���Ӷ�С��������ֵ�����ȶ�
		if (false == sTemp->iMark && DiffInput <= sTemp->sTempAuto.noiseBand)
		{
			sTemp->iMark = true;
		}
		else if(DiffInput > sTemp->sTempAuto.noiseBand)
		{
			sTemp->iMark = false;
			sTemp->iLastTimes = *(sTemp->sTempNor.Current_time);
		}
		
		if (sTemp->iAutoTimes - sTemp->iLastTimes > sTemp->iOKTimes && true == sTemp->iMark)
		{
			return true;
		}
	}
	return false;
}

//  PID���㺯��
int PID_Operation(TEMP_PID_STRUCT *sTemp)
{
	int iErrorCode = 0x05;
	bool bOK = false;
	double Diff = 0.0;
	Diff = *(sTemp->inputRun) - *(sTemp->setpointRun);
	// �����
	if(Diff >= *(sTemp->setpointRun) / 2)
	{
		sTemp->iMark = false;
		iErrorCode = 0x05;
		*(sTemp->outputRun) = sTemp->sTempNor.outMin;
		return iErrorCode;
	}

	if (sTemp->tuning)
	{
		// �����Զ���г���к��������ر�־λ�� �����ʾ�ɹ�
		int val = 0;
		val = (Runtime(&(sTemp->sTempAuto)));
		// ����ɹ����ر��Զ���г
		if (val != 0)
		{
			sTemp->iLastTimes = *(sTemp->sTempNor.Current_time);
			sTemp->iAutoTimes = 0;
			sTemp->iTuneMark = true;  // JTJ
			sTemp->tuning = false;
		}
		if (!sTemp->tuning)
		{ 
			double KP = GetKpAuto(&(sTemp->sTempAuto));
			double KI = GetKiAuto(&(sTemp->sTempAuto));
			double KD = GetKdAuto(&(sTemp->sTempAuto));
			SetTunings(KP, KI, KD, &(sTemp->sTempNor));
			AutoTuneHelper(false, sTemp);
		}
	}
	else
	{
		// �ж��Ƿ�����𵴣�������ִ��PID����
		if (false == sTemp->iTuneMark)
		{
			sTemp->tuning = SinTune(sTemp);
			// �趨�Զ���г����
			if (sTemp->tuning)
			{
				sTemp->tuning = false;
				changeAutoTune(sTemp);
				sTemp->tuning = true;
			}
		}
		Compute(&(sTemp->sTempNor));
		bOK =  PID_OK(sTemp);
	}
	
	// ��ֵռ�ձ�
	*(sTemp->outputRun) = (unsigned int)*(sTemp->outputRun);

	if (*(sTemp->outputRun) < 0 || *(sTemp->inputRun) < 0 || *(sTemp->setpointRun)< 0)
	{
		*(sTemp->outputRun) = sTemp->sTempNor.outMin;
	}
	
	// �����ȶ��ڣ����Ըı��趨ֵ
	if(true == bOK)
	{
		iErrorCode = 0x00;
		return iErrorCode;
	}
	if( true == sTemp->iTuneMark)	// �Զ������ڽ����� �����Ըı��趨ֵ
	{
		iErrorCode = 0x01;
		return iErrorCode;
	}
	if(true == sTemp->tuning)	// �Զ������ڣ����ɸı�
	{
		iErrorCode = 0x03;
		return iErrorCode;
	}
	if(true == sTemp->iMark)	// ���ڣ����ɸı�
	{
		iErrorCode = 0x03;
		return iErrorCode;
	}
	iErrorCode = 0x01;
	return iErrorCode;
}
