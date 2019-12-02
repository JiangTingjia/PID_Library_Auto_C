/**********************************************************************************************
*  ˵�������ļ�PID�ֶο���
*  ���ߣ���ͥ��
*  ���䣺kyzy_duck@163.com
*  ���룺GBK2312
**********************************************************************************************/

#include "PID_v1.h"
#include "math.h"

#define PIN_INPUT 0
#define PIN_OUTPUT 3

// ���ز���
double Setpoint = 0;
double Input = 0;
double Output = 0;
unsigned long time = 0;

// ��1PID���Ʋ���
double aggKp = 4;
double aggKi = 0.2;
double aggKd = 1;
// ��2PID���Ʋ���
double consKp = 1;
double consKi = 0.05;
double consKd = 0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT, &time);

void Init()
{
    // ����Ŀ��ֵ�������Լ�����Ҫ����ͬ��ֵ
    Setpoint = 100;
    // ����PID����
    myPID.SetMode(AUTOMATIC);
}

void main()
{
    // �趨PID��������������
    Init();

    // ����ϵͳʱ�䣬�����ڶ�ʱ�������
    time++;

    // ѭ����������
    while (1)
    {
        // ��������ֵ����ͬϵͳ��ʽ��һ������֮��Ҫ����������ֵ
        Input;
        double gap = fabs(Setpoint - Input);
        
        //�ֶθı���Ʋ���
        if (gap < 10)
        {
            myPID.SetTunings(consKp, consKi, consKd);
        }
        else
        {
            myPID.SetTunings(aggKp, aggKi, aggKd);
        }
        // ����PID�������������ֵ
        myPID.Compute();
        // ʹ��output���п���
        Output;
    }
}
