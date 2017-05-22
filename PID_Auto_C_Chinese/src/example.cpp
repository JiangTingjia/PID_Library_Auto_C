#include "include\PID_Nor.h"
#include "include\PID_Auto.h"
#include <stdbool.h>

typedef struct
{
	// ��ͨPID���Ʋ���
	double KP;			//Kp����
	double KI;			//Ki����
	double KD;			//Kd����
	double dMin;		// ����
	double dMax;		// ����
	unsigned int iSampleTimeNor;		// PID���ڲ���ʱ��
	int iDirection;						// PID���ڷ���
	int iMode;							//PID��������
	//�Զ���г����
	double aTuneStep;					// �Զ�����PWM�����ֵ
	double aTuneNoise;					//������ֵ
	double aTuneStartValue;				// �Զ���г�����׼ֵ����ʱ��ʹ�ã��������ã�
	unsigned int aTuneLookBack;		//����ʱ��
	int iType;						// PI���� 0   PID���� 1
}PID_INIT_PARM;

typedef struct
{
	// ���Ʋ���
	double *inputRun;		// ����		
	double *outputRun;		// ���
	double *setpointRun;	// �趨ֵ
	TEMP_NOR_STRUCT sTempNor;	// ��ͨPID���Ʋ���
	TEMP_AUTO_STRUCT sTempAuto;	//�Զ���г���Ʋ���
								// �жϲ���
	int ATuneModeRemember;	// PID������־��AUTOMATIC��MANUAL
	bool tuning;			// �Զ���г������־λ
	bool iMark;			//  �ж��񵴱�־λ
	bool iTuneMark;		// �Զ���г�Ƿ������־
}TEMP_PID_STRUCT;

int PID_Operation(TEMP_PID_STRUCT *sTemp);
void setupPID(TEMP_PID_STRUCT *sTemp, double *iCurVal, double *iSetVal, double *iOutput,
	unsigned int *iCurTime);
void setupParm(TEMP_PID_STRUCT *sTemp, PID_INIT_PARM sParm);


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
		//*****************************ע��*********************//
		// �����StartVal����趨Ϊ�������ں�2�������ڵ� output�ľ�ֵ, Step = StartVal
		// �Զ������ڼ䣬output���� StartVal - Step �� StartVal + Step,�������������ѡȡ�����ʣ�input�ᷢɢ
		double StartVal = *(sTemp->sTempNor.myOutput);
		SetStartOutput(StartVal, &(sTemp->sTempAuto));
		// �˴�Ϊ�Զ����εİ��ֵ�趨
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

/**********************************����***************************/
/*------------------------------------------------------------------------------
*Function: PID_Operation
* Brief: �˺������ڵ���PID���Ƶ�����
* Param:
*		sTemp	IN	 PID���ƶ���ʵ��
* Retval:  ������
*		:  bit0  �Ƿ�����ȶ���
*          bit1  �Ƿ���Ըı��趨ֵ
*		   bit2  �¶��Ƿ񳬹�
* 		   (1��ʾ��OK  0 ��ʾOK)
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
int PID_Operation(TEMP_PID_STRUCT *sTemp)
{
	int iErrorCode = 0;
	bool bOK = false;
	double Diff = 0.0;
	Diff = *(sTemp->inputRun) - *(sTemp->setpointRun);

	// ���ϵͳû�н����𵴣���ôִ��Compute������������PID���ڣ�Ҳ��������nor����
	// �������ϵͳ�������𵴣���ô�� sTemp->iTuneMark ��Ϊ true�� changeAutoTune����Ϊ�����Զ����εĳ�ʼ������������Ŀǰ����Ϊ�趨�ģ�����к��ʵķ����ж��Ƿ�����𵴣���ô���޸ĳ��Զ����룩
	// �ȴ����ν�����ϵͳ�����PID��������������nor��PID����
	if (sTemp->tuning)
	{
		// �����Զ���г���к��������ر�־λ�� �����ʾ�ɹ�
		int val = 0;
		val = (Runtime(&(sTemp->sTempAuto)));
		// ����ɹ����ر��Զ���г
		if (val != 0)
		{
			sTemp->iTuneMark = true;  
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
		// 
		if (false == sTemp->iTuneMark)
		{
			changeAutoTune(sTemp);
		}
		Compute(&(sTemp->sTempNor));
	}

	// ��ֵռ�ձ�
	*(sTemp->outputRun) = (unsigned int)*(sTemp->outputRun);

	if (*(sTemp->outputRun) < 0 || *(sTemp->inputRun) < 0 || *(sTemp->setpointRun)< 0)
	{
		*(sTemp->outputRun) = sTemp->sTempNor.outMin;
	}

	// �����ȶ��ڣ����Ըı��趨ֵ
	if (true == bOK)
	{
		iErrorCode = 0x00;
		return iErrorCode;
	}
	if (true == sTemp->iTuneMark)	// �Զ������ڽ����� �����Ըı��趨ֵ
	{
		iErrorCode = 0x01;
		return iErrorCode;
	}
	if (true == sTemp->tuning)	// �Զ������ڣ����ɸı�
	{
		iErrorCode = 0x03;
		return iErrorCode;
	}
	if (true == sTemp->iMark)	// ���ڣ����ɸı�
	{
		iErrorCode = 0x03;
		return iErrorCode;
	}
	iErrorCode = 0x01;
	return iErrorCode;
}

/*------------------------------------------------------------------------------
*Function: setupPID
* Brief: �˺�����ʼ��PID������
* Param:
*		sTemp	IN	 PID���ƶ���ʵ��
*		iCurVal	IN	 ����ֵ
*		iSetVal	IN	 �趨ֵ
*		iOutput	IN	 ���ֵ
*		iCurTime	IN	 ϵͳʱ��
* Retval:  ��
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
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

/*------------------------------------------------------------------------------
*Function: PID_Operation
* Brief: �˺������ڵ���PID���Ƶ�����
* Param:
*		sTemp	IN	 PID���ƶ���ʵ��
*		sParm	IN	 PID����

* Retval:  ��
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
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
	SetLookbackSec((int)(sParm.aTuneLookBack), &(sTemp->sTempAuto));	// �趨��������
}