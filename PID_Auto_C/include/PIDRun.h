/*---------------------------------------------------------------------------------------
*Author:JiangTingjia(kyzy_duck@163.com)
*Version:v1.0
*File:PID_Run.h
*Note: This file is used for PID process (Combine with PID control process and autotune process)
----------------------------------------------------------------------------------------*/
#include "PIDTemp.h"

typedef struct
{
	double dSetVal;
	double dCurVal;
	double dOutPut;
}PID_PARM_THREE;

// system time
extern unsigned int Current_time;

// Return Value for Runnning Status
extern int iErrorCodeFYP;
extern int iErrorCodeR1;
extern int iErrorCodeR2;

// instance FYP
extern TEMP_PID_STRUCT sFYP;
extern PID_PARM_THREE sFYP_PARM;
// instance R1
extern TEMP_PID_STRUCT sR1;
extern PID_PARM_THREE sR1_PARM;

// initial FYP
void PID_Setup_FYP(void);
// initial R1
void PID_Setup_R1(void);

// Calculate Output  CV
int PID_Cal(TEMP_PID_STRUCT *sTemp, double dSetVal, double dCurVal);




