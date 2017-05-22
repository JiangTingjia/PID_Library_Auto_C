#include "include\PID_Nor.h"
#include "include\PID_Auto.h"
#include <stdbool.h>

// 用来记忆自动调参前NOR的工作模式 是 PI  还是 PID
unsigned int ATuneModeRemember = 2;
// 定义
double input = 80, output = 50, setpoint = 180;
double kp = 2, ki = 0.5, kd = 2;
// 自动调参参数
double aTuneStep = 50, aTuneNoise = 1, aTuneStartValue = 100;
unsigned int aTuneLookBack = 20;
// 自动调参是否开启的标志位
bool tuning = false;

TEMP_NOR_STRUCT sTempNor;
TEMP_AUTO_STRUCT sTempAuto;

// 将iCurTime 链接到系统的运行时间
unsigned int iCurTime = 0.0;

// 恢复工作模式
void AutoTuneHelper(bool start)
{
	if (start)
		ATuneModeRemember = GetMode(&sTempNor);
	else
		SetMode(ATuneModeRemember, &sTempNor);
}

// 转换自动调参状态
void changeAutoTune()
{
	if (!tuning)
	{
		//Set the output to the desired starting frequency.
		output = aTuneStartValue;
		SetNoiseBand(aTuneNoise, &sTempAuto);
		SetOutputStep(aTuneStep, &sTempAuto);
		SetLookbackSec((int)aTuneLookBack, &sTempAuto);
		AutoTuneHelper(true);
		tuning = true;
	}
	else
	{ //cancel autotune
		Cancel(&sTempAuto);
		tuning = false;
		AutoTuneHelper(false);
	}
}



// 初始化
void setup()
{
	PID(&input, &output, &setpoint, &iCurTime, &sTempNor);		//初始化PID调节器，正向调节
	PID_ATune(&input, &output, &setpoint, &iCurTime, &sTempAuto);

	//Setup the pid 
	SetMode(AUTOMATIC, &sTempNor);

	if (tuning)
	{
		tuning = false;
		changeAutoTune();
		tuning = true;
	}
}

// 主流程
void mian()
{
	while (1)
	{
		if (tuning)
		{
			unsigned int val = (Runtime(&sTempAuto));
			if (val != 0)
			{
				tuning = false;
			}
			if (!tuning)
			{ 
				kp = GetKpNor(&sTempNor);
				ki = GetKiNor(&sTempNor);
				kd = GetKdNor(&sTempNor);
				SetTunings(kp, ki, kd, &sTempNor);
				AutoTuneHelper(false);
			}
		}
		else
		{
			Compute(&sTempNor);
		}

	}
}