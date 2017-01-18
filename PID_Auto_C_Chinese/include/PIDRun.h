
#include "PIDTemp.h"

typedef struct
{
	double dSetVal;
	double dCurVal;
	double dOutPut;
}PID_PARM_THREE;

// ϵͳʱ��
extern unsigned int Current_time;

// ������
extern int iErrorCodeFYP;
extern int iErrorCodeR1;
extern int iErrorCodeR2;

// ʵ��1
extern TEMP_PID_STRUCT sFYP;
extern PID_PARM_THREE sFYP_PARM;
// ʵ��2
extern TEMP_PID_STRUCT sR1;
extern PID_PARM_THREE sR1_PARM;
// ʵ��3
extern TEMP_PID_STRUCT sR2;
extern PID_PARM_THREE sR2_PARM;

// ��ʼ��FYP
void PID_Setup_FYP(void);
//��ʼ��R1
void PID_Setup_R1(void);
//��ʼ��R2
void PID_Setup_R2(void);

// ִ��
int PID_Cal(TEMP_PID_STRUCT *sTemp, double dSetVal, double dCurVal);




