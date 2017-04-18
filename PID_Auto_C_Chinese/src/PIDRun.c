/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PIDRun.c
*Note: ���ļ���ɳ�ʼ����������Ҫ�޸ĳ�ʼ��ʵ���б�Ҫ����
----------------------------------------------------------------------------------------*/
#include "../include/PIDRun.h"
// ������ ��һλ���� �Ƿ��ȶ��� �ڶ�λ�����Ƿ���Ըı��趨ֵ��  ����λ�����¶��Ƿ��쳣
// 0:��ʾ�ȶ������Ըı䡢���쳣 1����ʾδ�ȶ������ɸı䡢�¶��쳣
int iErrorCodeFYP = 0;
int iErrorCodeR1 = 0;

//ϵͳʱ�� ms
unsigned int Current_time;
// ʵ��1
TEMP_PID_STRUCT sFYP = { 0 };
PID_PARM_THREE sFYP_PARM = { 0 };
double dFYP_MaxOutPut = 0.0;
// ʵ��2
TEMP_PID_STRUCT sR1 = { 0 };
PID_PARM_THREE sR1_PARM = { 0 };
double dR1_MaxOutPut = 0.0;

// ��ʼ��ʵ��1
void PID_Setup_FYP(void)
{
	PID_INIT_PARM sInitParmFYP = { 0 };
	// �˴����ɴ����������
	{
		// �趨ʵ��1��Ӧ��PWM�������
		dFYP_MaxOutPut = 255;
		sInitParmFYP.dMin = 0;
		// �趨PID��������
		sInitParmFYP.iSampleTimeNor = 150;
		sInitParmFYP.iDirection = DIRECT;
		// �Զ�������ֵ�����ϵͳ���У������˽����趨ֵ���
		// ������΢����һ�������PV��Ѹ�ٱ仯����ô��Ҫ����ֵ��󣬴˴������趨Ϊ4���Ĵ���������
		sInitParmFYP.aTuneNoise = 5;
		sInitParmFYP.iType = 1;
		// �ڿ�����3���Ӻ�����Զ����Σ����ϵͳ����������ô�����ʵ���С�˲���������˲�������Ϊ0����ôϵͳ�Ͳ�������Զ�����
		sInitParmFYP.iSinTimes = 180000;
		// �Զ����ν���������2���� PV����ֵ��Χ�ڣ����ʾ�ȶ����������Ϊ0����ô������Ϊϵͳ�ȶ�
		sInitParmFYP.iOKTimes = 120000;
	}

	// ����dCurVal �� dSetVal����û�����壬ͨ��PID_Cal�ӿڸ���
	sFYP_PARM.dCurVal = 0;
	sFYP_PARM.dOutPut = 0;
	sFYP_PARM.dSetVal = 2000;
	sInitParmFYP.dMax = dFYP_MaxOutPut;

	// �趨��ʼ��KP/KI/KD
	sInitParmFYP.KP = 0.05 * sInitParmFYP.dMax;
	sInitParmFYP.KI = 0.01;
	sInitParmFYP.KD = 0;
	// ����PID���Ʊ�־
	sInitParmFYP.iMode = AUTOMATIC;
	// �������������Ҫ�޸�
	sInitParmFYP.aTuneLookBack = 5;
	sInitParmFYP.aTuneStartValue = 0;
	sInitParmFYP.aTuneStep = 0;
	setupPID(&sFYP, &(sFYP_PARM.dCurVal), &(sFYP_PARM.dSetVal), &(sFYP_PARM.dOutPut), &Current_time);
	setupParm(&sFYP, sInitParmFYP);
}

// ��ʼ��ʵ��2
void PID_Setup_R1(void)
{
	PID_INIT_PARM sInitParmR1 = { 0 };
	// �������
	dR1_MaxOutPut = 255;
	sR1_PARM.dCurVal = 0;
	sR1_PARM.dOutPut = 0;
	// �趨ֵ
	sR1_PARM.dSetVal = 2000;
	sInitParmR1.dMin = 0;
	sInitParmR1.dMax = dR1_MaxOutPut;
	// PID��������
	sInitParmR1.iSampleTimeNor = 50;
	sInitParmR1.iDirection = DIRECT;
	sInitParmR1.KP = 0.05 * sInitParmR1.dMax;
	sInitParmR1.KI = 0;
	sInitParmR1.KD = 0;
	sInitParmR1.iMode = AUTOMATIC;
	sInitParmR1.aTuneLookBack = 3;
	sInitParmR1.aTuneNoise = 50;
	sInitParmR1.aTuneStartValue = 0;
	sInitParmR1.aTuneStep = 0;
	sInitParmR1.iType = 0;
	sInitParmR1.iSinTimes = 30000;
	sInitParmR1.iOKTimes = 1000;
	setupPID(&sR1, &(sR1_PARM.dCurVal), &(sR1_PARM.dSetVal), &(sR1_PARM.dOutPut), &Current_time);
	setupParm(&sR1, sInitParmR1);
}


int PID_Cal(TEMP_PID_STRUCT *sTemp, double dSetVal, double dCurVal)
{
	int iErrorCode = 0;
	*(sTemp->inputRun) = dCurVal;
	*(sTemp->setpointRun) = dSetVal;
	iErrorCode =  PID_Operation(sTemp);
	return iErrorCode;
}


//*********************END*********************//
