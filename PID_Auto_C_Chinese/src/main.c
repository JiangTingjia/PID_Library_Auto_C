/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PIDRun.c
*Note: 此文件为外部交互代码，在合适的地方按照此文件格式，进行调用即可
----------------------------------------------------------------------------------------*/
#include "../include/PIDRun.h"

double dCurVal = 0.0;
double dSetVal = 2000.0;
// 输出 CV
double dOutputFYP = 0.0;
double dOutputR1 = 0.0;
int main()
{
	// 初始化实例对象
	PID_Setup_FYP();
	PID_Setup_R1();
	while (1)
	{
		// 计算输出
		iErrorCodeFYP = PID_Cal(&sFYP, dSetVal, dCurVal);
		dOutputFYP = *(sFYP.outputRun);
		// 获取输出
		iErrorCodeR1 = PID_Cal(&sR1, dSetVal, dCurVal);
		dOutputR1 = *(sR1.outputRun);
	}

	return 0;
}

