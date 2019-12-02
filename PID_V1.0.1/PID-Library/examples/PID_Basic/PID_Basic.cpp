/**********************************************************************************************
*  ˵�������ļ����PID������
*  ���ߣ���ͥ��
*  ���䣺kyzy_duck@163.com
*  ���룺GBK2312
**********************************************************************************************/

#include "PID_v1.h"

// ���ز���
double Setpoint = 0;
double Input = 0;
double Output = 0;
unsigned long time = 0;

//   PID�������Ʋ���
double Kp = 2;
double Ki = 5;
double Kd = 1;

// ���캯��
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT, &time);

void Init()
{
  // ����Ŀ��ֵ�������Լ�����Ҫ����ͬ��ֵ
  Setpoint = 100;
  // ����PID����
  myPID.SetMode(AUTOMATIC);
}

// main
void main(void)
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
    // ����PID�������������ֵ
    myPID.Compute();
    // ʹ��output���п���
    Output;
  }
}
