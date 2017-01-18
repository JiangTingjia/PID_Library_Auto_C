/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Auto.h
*Note: ���ļ���� PID �Զ����α�Ҫ���ܣ�����Ҫ�޸�
----------------------------------------------------------------------------------------*/
#ifndef PID_Auto
#define PID_Auto

#include "stdbool.h"

typedef struct
{
	/***********************************����****************************/
	// ���趨�ı���
	double noiseBand;			// ��������
	int controlType;			// ����ģʽ��־�� 1����PID���ƣ���������PI����
	int nLookBack;		// ���ݵ�����
	double oStep;		// ��ʾ���PWM���ֵ
	double outputStart;		// PWM ��ֵ
	// �������������
	unsigned int *Current_time;		//ϵͳ��ǰ����ʱ��
	double *input, *output, *setpoint;		// �������ͱ��������趨ֵ
	bool isMax, isMin;	// �Ƿ����ֵ����Сֵ�ı�־λ
	bool running;				// �Զ������Ƿ����б�־λ
	unsigned int peak1, peak2, lastTimeAuto;	// ���ڴ���嶥���ֵ�ʱ��
	unsigned int sampleTime;		// ����ʱ��
	int peakType;		// ����ֵ -1 0 1 �� -1 �����ȣ� 0 ����δ�з�ֵ��  1����嶥
	double lastInputs[101];		// ���ڴ洢��������ʷ�仯
	double peaks[10];		// ���ڴ洢�ο�ֵ
	int peakCount;		// ���ڴ洢��ֵ����
	bool justchanged;	// �Ƿ���ڷ�ֵת����־λ
	bool justevaled;	// �Ƿ�ʼ�����־λ
	double absMax, absMin;	// ���ֵ����Сֵ�ľ���ֵ
	double Ku, Pu;	// PID�����ĵ�г����
}TEMP_AUTO_STRUCT;

/**********************************����***************************/
/*------------------------------------------------------------------------------
*Function: PID_ATune
* Brief: �˺������ڳ�ʼ���Զ�����PID������
* Param: Input  IN  ��������
*        Output  IN  ������
* Retval:  �޷���ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void PID_ATune(double* Input, double* Output, double *setpoint, unsigned int *iCurTime, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: Runtime
* Brief: �˺������������Զ���г����
* Param: sTempAuto	IN �Զ���г����ʵ��
* Retval:  0���ɹ�
*		   1��ʧ��
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
int Runtime(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: Cancel
* Brief: �˺�������ȡ���Զ���гPID����������
* Param: sTempAuto	IN �Զ���г����ʵ��
* Retval:  �޷���ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void Cancel(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetOutputStep
* Brief: �˺��������趨�Զ���г�����PWM���ֵ
* Param: Step  IN  ���ֵ
*        sTempAuto	IN �Զ���г����ʵ��
* Retval:  �޷���ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetOutputStep(double Step, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetOutputStep
* Brief: �˺������ڻ�ȡ�Զ���г�����PWM���ֵ
* Param:   sTempAuto	IN �Զ���г����ʵ��
* Retval:  ���ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
double GetOutputStep(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetStartOutput
* Brief: �˺��������趨PWM��׼ֵ
* Param: StartVal  IN  ��������
*        sTempAuto	IN �Զ���г����ʵ��
* Retval:  �޷���ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetStartOutput(double StartVal, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GettStartOutput
* Brief: �˺������ڻ�ȡPWM��׼ֵ
* Param: 
*        sTempAuto	IN �Զ���г����ʵ��
* Retval:  ��׼ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
double GettStartOutput(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetControlType
* Brief: �˺��������趨����ģʽ
* Param: Type  IN  1��ʾPID���ƣ�  0��ʾPI����
*        sTempAuto	IN �Զ���г����ʵ��
* Retval:  �޷���ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetControlType(int Type, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetControlType
* Brief: �˺������ڻ�ȡPID����������ģʽ
* Param:
*        sTempAuto	IN �Զ���г����ʵ��
* Retval:  0��PI
*		   1��PID
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
int GetControlType(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetLookbackSec
* Brief: �˺��������趨����ʱ��
* Param: value  IN  ����ʱ��
*        sTempAuto	IN �Զ���г����ʵ��
* Retval:  �޷���ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetLookbackSec(int value, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetLookbackSec
* Brief: �˺������ڻ�ȡ����ʱ��
* Param:
*        sTempAuto	IN �Զ���г����ʵ��
* Retval:  ����ʱ��
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
int GetLookbackSec(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: SetNoiseBand
* Brief: �˺��������趨��������
* Param: Band  IN  ��������
*        sTempAuto	IN �Զ���г����ʵ��
* Retval:  �޷���ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetNoiseBand(double Band, TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetNoiseBand
* Brief: �˺������ڻ�ȡ��������
* Param:
*        sTempAuto	IN �Զ���г����ʵ��
* Retval:  ��������
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
double GetNoiseBand(TEMP_AUTO_STRUCT *sTempAuto);

/*------------------------------------------------------------------------------
*Function: GetKpAuto/GetKiAuto/GetKdAuto
* Brief: �˺������ڻ�ȡPID���Ʋ���
* Param: 
*        sTempAuto	IN �Զ���г����ʵ��
* Retval:  Kp/kI/Kd
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
double GetKpAuto(TEMP_AUTO_STRUCT *sTempAuto);										
double GetKiAuto(TEMP_AUTO_STRUCT *sTempAuto);										
double GetKdAuto(TEMP_AUTO_STRUCT *sTempAuto);										

/*------------------------------------------------------------------------------
*Function: FinishUp
* Brief: �˺������ڵ��ڽ��������PID�����м����
* Param: 
*        sTempAuto	IN �Զ���г����ʵ��
* Retval:  �޷���ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void FinishUp(TEMP_AUTO_STRUCT *sTempAuto);


#endif

