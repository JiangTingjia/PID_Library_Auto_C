/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:main.c
*Note: thie file is a example for PID control
----------------------------------------------------------------------------------------*/
#include "PIDRun.h"
//  SP PV
double dCurVal = 0.0;
double dSetVal = 2000.0;
//  CV
double dOutputFYP = 0.0;
double dOutputR1 = 0.0;
int main()
{
	PID_Setup_FYP();
	PID_Setup_R1();
	while (1)
	{
		dSetVal = 2000;
		iErrorCodeFYP = PID_Cal(&sFYP, dSetVal, dCurVal);
		dOutputFYP = *(sFYP.outputRun);
		iErrorCodeR1 = PID_Cal(&sR1, dSetVal, dCurVal);
		dOutputR1 = *(sR1.outputRun);
	}

	return 0;
}

