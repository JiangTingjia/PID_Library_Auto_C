/**********************************************************************************************
*  ˵�������ļ����PID�������ƣ�������������������趨 
*  ���ߣ���ͥ��
*  ���䣺kyzy_duck@163.com
*  ���룺GBK2312
**********************************************************************************************/
#ifndef __PID_V1_H__
#define __PID_V1_H__

#ifndef PID_VERSION
#define PID_VERSION 1.0.1
#endif

/*******************�궨��*******************/
#ifndef AUTOMATIC
#define AUTOMATIC 1 // ����PID����
#endif

#ifndef MANUAL
#define MANUAL 0 // �ر�PID����
#endif

#ifndef DIRECT
#define DIRECT 0 // ������ƣ��������������������
#endif

#ifndef REVERSE
#define REVERSE 1 //������ƣ�����������¸�������
#endif

#ifndef P_ON_M
#define P_ON_M 0 // ����ӱ�������
#endif

#ifndef P_ON_E
#define P_ON_E 1 // ��ӱ�������
#endif

#ifdef __cplusplus
extern "C"
{
#endif
	class PID
	{
	public:
		/*------------------------------------------------------------------------------
		* description: ���캯��
		* params:  IN Input       ����ֵ
		*         IN Output      ���ֵ
		*         IN Setpoint  	 �趨ֵ
		*         IN Kp  	     K��������
		*         IN Ki        	 I���ֲ���
		*         IN Kd      	 D΢�ֲ���
		*		  IN POn         �Ƿ����������ƣ�P_ON_M������ӱ������ƣ�P_ON_E����ӱ�������
		*         IN ControllerDirection   ���Ʒ���  DIRECT��������ƣ�REVERSE���������
		*		  IN time		 ����ϵͳʱ�䣬����ȷ���Ƿ������������
		* return:  None
		----------------------------------------------------------------------------------*/
		PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki,
			double Kd, int POn, int ControllerDirection, void *time);
		PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki,
			double Kd, int ControllerDirection, void *time);

		/*------------------------------------------------------------------------------
		* description:  �趨/��ȡPID����״̬
		* params:  IN Mode       AUTOMATIC�������� MANUAL���ر�
		* return:  �����ر�״̬
		----------------------------------------------------------------------------------*/
		void SetMode(int Mode);
		int GetMode();

		/*------------------------------------------------------------------------------
		* description:  PID�������̣��˺���Ӧ��ѭ�������ã�һ����������ܣ� ���ֵ�������£����ֵ���޶��� OutputLimits
		* ��Χ��
		* params:  None
		* return�� 1: ���ֵ�ɹ������£� 0: ���ֵδ������ 
		----------------------------------------------------------------------------------*/
		bool Compute();

		/*------------------------------------------------------------------------------
		* description:  �����趨/��ȡPID��ֵ�������С����
		* params:  IN Min  ��Сֵ
		*		   IN Max  ���ֵ
		* return: ��Ӧ�������
		----------------------------------------------------------------------------------*/
		void SetOutputLimits(double Min, double Max);
		double GetOutputMin(void);
		double GetOutputMax(void);

		/*------------------------------------------------------------------------------
		* description:  �趨/��ȡ PID ���Ʋ���
		* params:  IN Kp   P�������Ʋ���
		*		   IN Ki   I���ֿ��Ʋ���
		*		   IN Kd   d΢�ֿ��Ʋ���
		*   	   IN POn  �Ƿ����������ƣ�P_ON_M������ӱ������ƣ�P_ON_E����ӱ�������
		* return:  ��Ӧ PID ���Ʋ���
		----------------------------------------------------------------------------------*/
		void SetTunings(double v_Kp, double v_Ki, double v_Kd);
		void SetTunings(double v_Kp, double v_Ki, double v_Kd, int v_POn);
		double GetKp();
		double GetKi();
		double GetKd();
		int GetPonE();

		/*------------------------------------------------------------------------------
		* description:  �趨/��ȡ PID ���Ʒ���
		* params:  IN Direction   ���Ʒ���  DIRECT: �������,  REVERSE: �������
		* return:  ���Ʒ���
		----------------------------------------------------------------------------------*/
		void SetDirection(int Direction);
		int GetDirection();

		/*------------------------------------------------------------------------------
		* description:  �趨/��ȡ PID ��������
		* params:  IN NewSampleTime   ms
		* return:  �������� ms
		----------------------------------------------------------------------------------*/
		void SetSampleTime(unsigned int NewSampleTime);
		unsigned int GetSampleTime(void);

	private:
		void Initialize();

		double dispKp; // P ��������
		double dispKi; // I ���ֲ���
		double dispKd; // D ΢�ֲ���

		double kp;
		double ki;
		double kd;
		int controllerDirection;
		int pOn;
		double *myInput;
		double *myOutput;
		double *mySetpoint;

		unsigned long lastTime;
		double outputSum;
		double lastInput;

		unsigned int SampleTime;
		double outMin;
		double outMax;
		bool inAuto;
		bool pOnE;

		unsigned long *currentTime;
	};

#ifdef __cplusplus
}
#endif

#endif //__PID_V1_H__
