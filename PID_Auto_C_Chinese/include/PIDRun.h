
#include "PIDTemp.h"

typedef struct
{
	double dSetVal;
	double dCurVal;
	double dOutPut;
}PID_PARM_THREE;

// 系统时间
extern unsigned int Current_time;

// 错误码
extern int iErrorCodeFYP;
extern int iErrorCodeR1;
extern int iErrorCodeR2;

// 实例1
extern TEMP_PID_STRUCT sFYP;
extern PID_PARM_THREE sFYP_PARM;
// 实例2
extern TEMP_PID_STRUCT sR1;
extern PID_PARM_THREE sR1_PARM;
// 实例3
extern TEMP_PID_STRUCT sR2;
extern PID_PARM_THREE sR2_PARM;

// 初始化FYP
void PID_Setup_FYP(void);
//初始化R1
void PID_Setup_R1(void);
//初始化R2
void PID_Setup_R2(void);

// 执行
int PID_Cal(TEMP_PID_STRUCT *sTemp, double dSetVal, double dCurVal);




