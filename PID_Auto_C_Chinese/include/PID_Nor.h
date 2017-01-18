/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Nor.h
*Note: ���ļ����PID���ڵı�Ҫ���ܣ�����Ҫ�޸�
----------------------------------------------------------------------------------------*/
#ifndef PID_Nor_h
#define PID_Nor_h
#include "stdbool.h"

#define AUTOMATIC	1		//����PID����
#define MANUAL	0			//�ر�PID����
#define DIRECT  0			//�������
#define REVERSE  1			//�������

///***********************************����****************************/
typedef struct
{
	// ���趨�Ĳ���
	double kpNor;                  // Kp���� (����PID�����ʵ����)
	double kiNor;                  // Ki���� (����PID�����ʵ����)
	double kdNor;                  // Kd���� (����PID�����ʵ����)
	bool controllerDirection;		  // ���Ʒ��� 0 ����
	unsigned int SampleTime;	// ����ʱ�� ms
	double outMin, outMax;		// ������
	bool inAuto;					// �Ƿ���PID���Ʊ�ʶ 1����
	// ���й�������Ҫ�Ŀ��Ʋ���
	double *myInput;              //  ������
	double *myOutput;             //  ������
	double *mySetpoint;           //  �趨ֵ 
	unsigned int *Current_time;		//ϵͳ��ǰ����ʱ��
	unsigned int lastTimeNor;	 //  ��һ�μ���ʱ��Ӧ��ϵͳ����ʱ��
	double ITerm;				// ������
	double lastInput;			// ��һ�α�����
	// ��־λ
	double dispKp;				// Kp���� (Get����ʱ����ֵ)
	double dispKi;				// Ki���� (Get����ʱ����ֵ)
	double dispKd;				// Kd���� (Get����ʱ����ֵ)
}TEMP_NOR_STRUCT;

/**********************************����***************************/
/*------------------------------------------------------------------------------
*Function: PID
* Brief: �˺������ڳ�ʼ��PID������
* Param: Input  IN  ��������
*        Output  IN  ������
*        Setpoint  IN  �趨ֵ
*        Kp  IN  P����
*        Ki  IN  I����
*        Kd  IN  D����
*        ControllerDirection  IN  �������Ĺ�������
* Retval:  �޷���ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void PID(double* Input, double* Output, double* Setpoint, unsigned int *iCurTime, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: Initialize
* Brief: �˺��������޸�PID����ʱ��󣬲������³�ʼ�����ⲿ����Ҫ����
* Param: ��
* Retval: ��
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void Initialize(TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: Compute
* Brief: �˺������ڼ���PID�������������Ҳ���ǿ�����
* Param: ��
* Retval:  �ɹ���true
*		   ʧ�ܣ�false
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
bool Compute(TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetMode
* Brief: �˺������ڿ������߹ر�PID���� AUTOMATIC or MANUAL
* Param: Mode  IN  PID����������
* Retval:  �޷���ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetMode(int Mode, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: GetMode
* Brief: �˺������ڻ�ȡPID������ �Ƿ���
* Param: ��
* Retval: ����: 1
*         �رգ�0
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
int GetMode(TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetOutputLimits
* Brief: �˺��������趨PID��������������ޣ� Ĭ��Ϊ0��255
* Param: Min  IN  ����
*        Max  IN  ����
* Retval:  ��
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetOutputLimits(double Min, double Max, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetSampleTime
* Brief: �˺��������趨PID������ ����Ƶ�� �� Ĭ��Ϊ100ms
* Param: NewSampleTime  IN  ����Ƶ��(ms)
* Retval:  ��
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetSampleTime(unsigned int NewSampleTime, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetTunings
* Brief: �˺��������趨PID������ P I D����
* Param: Kp  IN  P����
*        Ki  IN  I����
*        Kd  IN  D����
* Retval:  ��
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetTunings(double Kp, double Ki, double Kd, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: GetKpNor
* Brief: �˺������ڻ�ȡPID������ ��ǰP����
* Param: ��
* Retval:  P����ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
double GetKpNor(TEMP_NOR_STRUCT *sTempNor);						  // These functions query the pid for interal values.

/*------------------------------------------------------------------------------
*Function: GetKiNor
* Brief: �˺������ڻ�ȡPID������ ��ǰI����
* Param: ��
* Retval:  I����ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
double GetKiNor(TEMP_NOR_STRUCT *sTempNor);						  //  they were created mainly for the pid front-end,

/*------------------------------------------------------------------------------
*Function: GetKdNor
* Brief: �˺������ڻ�ȡPID������ ��ǰD����
* Param: ��
* Retval:  D����ֵ
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
double GetKdNor(TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: SetControllerDirection
* Brief: �˺��������趨PID������ �������
* Param: Direction  IN  �����
* Retval:  ��
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
void SetControllerDirection(bool Direction, TEMP_NOR_STRUCT *sTempNor);

/*------------------------------------------------------------------------------
*Function: GetDirection
* Brief: �˺������ڻ�ȡPID������ ����
* Param: ��
* Retval: ����: 0
*         ����1
*Author: JiangTingjia(kyzy_duck@163.com)
----------------------------------------------------------------------------------*/
int GetDirection(TEMP_NOR_STRUCT *sTempNor);

#endif

