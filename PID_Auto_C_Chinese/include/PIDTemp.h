/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PIDTemp.h
*Note: ���ļ����PID���ڵĿ����������
----------------------------------------------------------------------------------------*/
#ifndef PID_TEMP_H
#define PID_TEMP_H
#include "PID_Nor.h"
#include "PID_Auto.h"

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
	// �����ж��Ƿ������
	unsigned int iSinTimes;  	// MS
	// �����ж��Ƿ��ȶ�
	unsigned int iOKTimes;		// MS
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
	bool iTuneMark ;		// �Զ���г�Ƿ������־
	// �𵴼��ȶ��жϱ�־�Ͳ���
	unsigned int iAutoTimes;	// �����ж��ǽ�����
	unsigned int iLastTimes;	// �����ж��ǽ�����
	double iSumOutput;
	unsigned int iTotalTimes;
	unsigned int iSinTimes;  	// MS
	unsigned int iOKTimes;		// MS
}TEMP_PID_STRUCT;

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
int PID_Operation(TEMP_PID_STRUCT *sTemp);

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
	unsigned int *iCurTime);

/*------------------------------------------------------------------------------
*Function: PID_Operation
* Brief: �˺������ڵ���PID���Ƶ�����
* Param: 
*		sTemp	IN	 PID���ƶ���ʵ��
*		sParm	IN	 PID����

* Retval:  ��
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void setupParm(TEMP_PID_STRUCT *sTemp, PID_INIT_PARM sParm);

#endif

