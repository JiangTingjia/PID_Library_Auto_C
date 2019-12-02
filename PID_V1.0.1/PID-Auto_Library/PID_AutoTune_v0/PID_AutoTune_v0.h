/**********************************************************************************************
*  说明：此文件完成PID自整定
*  作者：蒋庭佳
*  邮箱：kyzy_duck@163.com
*  编码：GBK2312
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
		* description: 构造函数
		* params:  IN Input       输入值
		*          IN Output      输出值
		*		   IN time		 挂载系统时间，用于确定是否满足计算周期
		* return:  None
		----------------------------------------------------------------------------------*/
		PID_ATune(double *Input, double *Output, void *time);

		/*------------------------------------------------------------------------------
		* description:  自整定过程函数，此函数应当被循环调用，在过程中，控制器会将输入震荡，用于寻找PID控制参数
		* params:  None
		* return： 1: 自整定成功标志 0: 自整定过程中 
		----------------------------------------------------------------------------------*/
		int Runtime();

		/*------------------------------------------------------------------------------
		* description:  主动取消自整定过程
		* params:  None
		* return： None
		----------------------------------------------------------------------------------*/
		void Cancel();

		/*------------------------------------------------------------------------------
		* description:  设定/获取 自整定震荡过程中输出值的半宽， 自整定过程中，数值值会在自整定开始的输出值基础上，按照此函数设定的半宽进行输出
		* params:  IN double Step 输出值半宽
		* return： 输出半宽
		----------------------------------------------------------------------------------*/
		void SetOutputStep(double Step);
		double GetOutputStep();

		/*------------------------------------------------------------------------------
		* description:  设定/获取 自整定最终计算的控制参数，
		* params:  IN double Step 输出值半宽
		* return： 输出半宽
		----------------------------------------------------------------------------------*/
		void SetControlType(int Type);
		int GetControlType();

		/*------------------------------------------------------------------------------
		* description:  设定/获取 震荡次数，震荡次数越多，精确度越高，但需要的时间越久，一般设置在3到9之间
		* params:  IN int value 震荡次数
		* return： 同上
		----------------------------------------------------------------------------------*/
		void SetLookbackSec(int value); 
		int GetLookbackSec();			

		/*------------------------------------------------------------------------------
		* description:  设定/获取 输出翻转阈值，当输入值大于或者小于设定值±此半宽时，输出进行反转
		* params:  IN double Band 反转阈值，根据输出值的范围进行设定
		* return： 同上
		----------------------------------------------------------------------------------*/
		void SetNoiseBand(double Band);
		double GetNoiseBand();

		/*------------------------------------------------------------------------------
		* description:  自整定结束后，通过此函数获取PID控制参数 
		* params:  None
		* return： PID控制参数
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
		// 鐢ㄤ簬鎸傚湪绯荤粺鏃堕棿
		unsigned long *currentTime;
	};

#ifdef __cplusplus
}
#endif

#endif //__PID_AUTOTUNE_V0_H__
