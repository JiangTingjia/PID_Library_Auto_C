/**********************************************************************************************
*  说明：此文件完成PID基本控制，包含控制器所有相关设定 
*  作者：蒋庭佳
*  邮箱：kyzy_duck@163.com
*  编码：GBK2312
**********************************************************************************************/
#ifndef __PID_V1_H__
#define __PID_V1_H__

#ifndef PID_VERSION
#define PID_VERSION 1.0.1
#endif

/*******************宏定义*******************/
#ifndef AUTOMATIC
#define AUTOMATIC 1 // 开启PID控制
#endif

#ifndef MANUAL
#define MANUAL 0 // 关闭PID控制
#endif

#ifndef DIRECT
#define DIRECT 0 // 正向控制，正的输出导致正的输入
#endif

#ifndef REVERSE
#define REVERSE 1 //反向控制，正的输出导致负的输入
#endif

#ifndef P_ON_M
#define P_ON_M 0 // 不添加比例控制
#endif

#ifndef P_ON_E
#define P_ON_E 1 // 添加比例控制
#endif

#ifdef __cplusplus
extern "C"
{
#endif
	class PID
	{
	public:
		/*------------------------------------------------------------------------------
		* description: 构造函数
		* params:  IN Input       输入值
		*         IN Output      输出值
		*         IN Setpoint  	 设定值
		*         IN Kp  	     K比例参数
		*         IN Ki        	 I积分参数
		*         IN Kd      	 D微分参数
		*		  IN POn         是否开启比例控制，P_ON_M：不添加比例控制，P_ON_E：添加比例控制
		*         IN ControllerDirection   控制方向  DIRECT：正向控制，REVERSE：反向控制
		*		  IN time		 挂载系统时间，用于确定是否满足计算周期
		* return:  None
		----------------------------------------------------------------------------------*/
		PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki,
			double Kd, int POn, int ControllerDirection, void *time);
		PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki,
			double Kd, int ControllerDirection, void *time);

		/*------------------------------------------------------------------------------
		* description:  设定/获取PID开启状态
		* params:  IN Mode       AUTOMATIC：开启， MANUAL：关闭
		* return:  开启关闭状态
		----------------------------------------------------------------------------------*/
		void SetMode(int Mode);
		int GetMode();

		/*------------------------------------------------------------------------------
		* description:  PID计算流程，此函数应当循环被调用，一旦满足计算周， 输出值即被更新，输出值被限定在 OutputLimits
		* 范围内
		* params:  None
		* return： 1: 输出值成功被更新， 0: 输出值未被更新 
		----------------------------------------------------------------------------------*/
		bool Compute();

		/*------------------------------------------------------------------------------
		* description:  用于设定/获取PID数值的最大最小限制
		* params:  IN Min  最小值
		*		   IN Max  最大值
		* return: 对应输出限制
		----------------------------------------------------------------------------------*/
		void SetOutputLimits(double Min, double Max);
		double GetOutputMin(void);
		double GetOutputMax(void);

		/*------------------------------------------------------------------------------
		* description:  设定/获取 PID 控制残数
		* params:  IN Kp   P比例控制参数
		*		   IN Ki   I积分控制参数
		*		   IN Kd   d微分控制参数
		*   	   IN POn  是否加入比例控制，P_ON_M：不添加比例控制，P_ON_E：添加比例控制
		* return:  对应 PID 控制参数
		----------------------------------------------------------------------------------*/
		void SetTunings(double v_Kp, double v_Ki, double v_Kd);
		void SetTunings(double v_Kp, double v_Ki, double v_Kd, int v_POn);
		double GetKp();
		double GetKi();
		double GetKd();
		int GetPonE();

		/*------------------------------------------------------------------------------
		* description:  设定/获取 PID 控制方向
		* params:  IN Direction   控制方向  DIRECT: 正向控制,  REVERSE: 反向控制
		* return:  控制方向
		----------------------------------------------------------------------------------*/
		void SetDirection(int Direction);
		int GetDirection();

		/*------------------------------------------------------------------------------
		* description:  设定/获取 PID 计算周期
		* params:  IN NewSampleTime   ms
		* return:  计算周期 ms
		----------------------------------------------------------------------------------*/
		void SetSampleTime(unsigned int NewSampleTime);
		unsigned int GetSampleTime(void);

	private:
		void Initialize();

		double dispKp; // P 比例参数
		double dispKi; // I 积分参数
		double dispKd; // D 微分参数

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
