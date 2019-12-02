/**********************************************************************************************
*  ˵�������ļ����PID������
*  ���ߣ���ͥ��
*  ���䣺kyzy_duck@163.com
*  ���룺GBK2312
**********************************************************************************************/
#ifndef __PID_AUTOTUNE_V0_H__
#define __PID_AUTOTUNE_V0_H__

#ifndef PID_AUTO_VERSION
#define PID_AUTO_VERSION 1.0.1
#endif

#ifdef __cplusplus
extern "C"
{
#endif

	class PID_ATune
	{
	public:
		/*------------------------------------------------------------------------------
		* description: ���캯��
		* params:  IN Input       ����ֵ
		*          IN Output      ���ֵ
		*		   IN time		 ����ϵͳʱ�䣬����ȷ���Ƿ������������
		* return:  None
		----------------------------------------------------------------------------------*/
		PID_ATune(double *Input, double *Output, void *time);

		/*------------------------------------------------------------------------------
		* description:  ���������̺������˺���Ӧ����ѭ�����ã��ڹ����У��������Ὣ�����𵴣�����Ѱ��PID���Ʋ���
		* params:  None
		* return�� 1: �������ɹ���־ 0: ������������ 
		----------------------------------------------------------------------------------*/
		int Runtime();

		/*------------------------------------------------------------------------------
		* description:  ����ȡ������������
		* params:  None
		* return�� None
		----------------------------------------------------------------------------------*/
		void Cancel();

		/*------------------------------------------------------------------------------
		* description:  �趨/��ȡ �������𵴹��������ֵ�İ�� �����������У���ֵֵ������������ʼ�����ֵ�����ϣ����մ˺����趨�İ��������
		* params:  IN double Step ���ֵ���
		* return�� ������
		----------------------------------------------------------------------------------*/
		void SetOutputStep(double Step);
		double GetOutputStep();

		/*------------------------------------------------------------------------------
		* description:  �趨/��ȡ ���������ռ���Ŀ��Ʋ�����
		* params:  IN double Step ���ֵ���
		* return�� ������
		----------------------------------------------------------------------------------*/
		void SetControlType(int Type);
		int GetControlType();

		/*------------------------------------------------------------------------------
		* description:  �趨/��ȡ �𵴴������𵴴���Խ�࣬��ȷ��Խ�ߣ�����Ҫ��ʱ��Խ�ã�һ��������3��9֮��
		* params:  IN int value �𵴴���
		* return�� ͬ��
		----------------------------------------------------------------------------------*/
		void SetLookbackSec(int value); 
		int GetLookbackSec();			

		/*------------------------------------------------------------------------------
		* description:  �趨/��ȡ �����ת��ֵ��������ֵ���ڻ���С���趨ֵ���˰��ʱ��������з�ת
		* params:  IN double Band ��ת��ֵ���������ֵ�ķ�Χ�����趨
		* return�� ͬ��
		----------------------------------------------------------------------------------*/
		void SetNoiseBand(double Band);
		double GetNoiseBand();

		/*------------------------------------------------------------------------------
		* description:  ������������ͨ���˺�����ȡPID���Ʋ��� 
		* params:  None
		* return�� PID���Ʋ���
		----------------------------------------------------------------------------------*/
		double GetKp();
		double GetKi();
		double GetKd();

	private:
		void FinishUp();
		bool isMax;
		bool isMin;
		double *input;
		double *output;
		double setpoint;
		double noiseBand;
		int controlType;
		bool running;
		unsigned long peak1;
		unsigned long peak2;
		unsigned long lastTime;
		int sampleTime;
		int nLookBack;
		int peakType;
		double lastInputs[101];
		double peaks[10];
		int peakCount;
		bool justchanged;
		bool justevaled;
		double absMax;
		double absMin;
		double oStep;
		double outputStart;
		double Ku;
		double Pu;
		// 用于挂在系统时间
		unsigned long *currentTime;
	};

#ifdef __cplusplus
}
#endif

#endif //__PID_AUTOTUNE_V0_H__
