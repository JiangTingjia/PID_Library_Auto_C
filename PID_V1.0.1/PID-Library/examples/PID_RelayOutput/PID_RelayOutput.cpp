/********************************************************
 * PID RelayOutput Example
 * Same as basic example, except that this time, the output
 * is going to a digital pin which (we presume) is controlling
 * a relay.  the pid is designed to Output an analog value,
 * but the relay can only be On/Off.
 *
 *   to connect them together we use "time proportioning
 * control"  it's essentially a really slow version of PWM.
 * first we decide on a window size (5000mS say.) we then
 * set the pid to adjust its output between 0 and that window
 * size.  lastly, we add some logic that translates the PID
 * output into "Relay On Time" with the remainder of the
 * window being "Relay Off Time"
 ********************************************************/
/**********************************************************************************************
*  ˵�������ļ����ģ�������Ϊ���������PID���������������������С��ֵ�У���������Ϳ��Ʋ�����������Ӧ�����
*  Ϊģ������Ȼ��������ʵ��Ӧ�ù����У�������Ҫ�������Ƽ̵�����������(0/1)
*  ���ߣ���ͥ��
*  ���䣺kyzy_duck@163.com
*  ���룺GBK2312
**********************************************************************************************/

#include "PID_v1.h"

#define PIN_INPUT 0
#define RELAY_PIN 6

//Define Variables we'll be connecting to
double Setpoint = 0;
double Input = 0;
double Output = 0;
unsigned long time = 0;

//Specify the links and initial tuning parameters
double Kp = 2;
double Ki = 5;
double Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT, &time);

int WindowSize = 5000;
unsigned long windowStartTime = 0;

// ��ʱ���ж�
void TIM()
{
    time++;
}

void Init()
{
    windowStartTime = time;

    // �趨ֵ�� ����Ŀ��ֵ�������Լ�����Ҫ����ͬ��ֵ
    Setpoint = 100;

    // �ı��������
    myPID.SetOutputLimits(0, WindowSize);

    //  ����PID����
    myPID.SetMode(AUTOMATIC);
}

void main()
{
    // ��ʼ������Ŀ��ֵ�������Լ�����Ҫ����ͬ��ֵ
    Init();

    // ѭ������
    while (1)
    {
        // ��������ֵ����ͬϵͳ��ʽ��һ������֮��Ҫ����������ֵ
        Input;
        // ����PID�������������ֵ
        myPID.Compute();
        // ʹ��output���п���
        // �ж��Ƿ񵽴����ʱ��
        if (time - windowStartTime > WindowSize)
        {
            windowStartTime += WindowSize;
        }
        // ��ƽ���
        if (Output < time - windowStartTime)
        {
            // ����ߵ�ƽ
        }
        else
        {
            // ����͵�ƽ
        }
    }
}
