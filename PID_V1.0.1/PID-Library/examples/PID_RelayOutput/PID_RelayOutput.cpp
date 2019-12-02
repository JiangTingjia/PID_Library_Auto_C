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
*  说明：此文件完成模拟输出变为数字输出，PID控制器的输出会在最大和最小限值中，根据输入和控制参数，调整对应输出，
*  为模拟量，然而在我们实际应用过程中，经常需要控制类似继电器的数字量(0/1)
*  作者：蒋庭佳
*  邮箱：kyzy_duck@163.com
*  编码：GBK2312
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

// 定时器中断
void TIM()
{
    time++;
}

void Init()
{
    windowStartTime = time;

    // 设定值， 设置目标值，根据自己的需要给不同的值
    Setpoint = 100;

    // 改变输出限制
    myPID.SetOutputLimits(0, WindowSize);

    //  开启PID控制
    myPID.SetMode(AUTOMATIC);
}

void main()
{
    // 初始化设置目标值，根据自己的需要给不同的值
    Init();

    // 循环调用
    while (1)
    {
        // 更新输入值，不同系统方式不一样，总之需要更新输输入值
        Input;
        // 调用PID控制器计算输出值
        myPID.Compute();
        // 使用output进行控制
        // 判断是否到达更新时间
        if (time - windowStartTime > WindowSize)
        {
            windowStartTime += WindowSize;
        }
        // 电平输出
        if (Output < time - windowStartTime)
        {
            // 输出高电平
        }
        else
        {
            // 输出低电平
        }
    }
}
