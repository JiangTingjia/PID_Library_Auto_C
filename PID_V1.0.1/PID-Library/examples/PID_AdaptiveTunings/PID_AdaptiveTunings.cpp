/**********************************************************************************************
*  说明：此文件PID分段控制
*  作者：蒋庭佳
*  邮箱：kyzy_duck@163.com
*  编码：GBK2312
**********************************************************************************************/

#include "PID_v1.h"
#include "math.h"

#define PIN_INPUT 0
#define PIN_OUTPUT 3

// 挂载参数
double Setpoint = 0;
double Input = 0;
double Output = 0;
unsigned long time = 0;

// 段1PID控制参数
double aggKp = 4;
double aggKi = 0.2;
double aggKd = 1;
// 段2PID控制参数
double consKp = 1;
double consKi = 0.05;
double consKd = 0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT, &time);

void Init()
{
    // 设置目标值，根据自己的需要给不同的值
    Setpoint = 100;
    // 开启PID控制
    myPID.SetMode(AUTOMATIC);
}

void main()
{
    // 设定PID控制器基本参数
    Init();

    // 增加系统时间，可以在定时器中完成
    time++;

    // 循环计算流程
    while (1)
    {
        // 更新输入值，不同系统方式不一样，总之需要更新输输入值
        Input;
        double gap = fabs(Setpoint - Input);
        
        //分段改变控制参数
        if (gap < 10)
        {
            myPID.SetTunings(consKp, consKi, consKd);
        }
        else
        {
            myPID.SetTunings(aggKp, aggKi, aggKd);
        }
        // 调用PID控制器计算输出值
        myPID.Compute();
        // 使用output进行控制
        Output;
    }
}
