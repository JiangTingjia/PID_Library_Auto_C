/**********************************************************************************************
*  说明：此文件完成PID自整定
*  作者：蒋庭佳
*  邮箱：kyzy_duck@163.com
*  编码：GBK2312
**********************************************************************************************/

#include "PID_v1.h"

// 挂载参数
double Setpoint = 0;
double Input = 0;
double Output = 0;
unsigned long time = 0;

//   PID基本控制参数
double Kp = 2;
double Ki = 5;
double Kd = 1;

// 构造函数
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT, &time);

void Init()
{
  // 设置目标值，根据自己的需要给不同的值
  Setpoint = 100;
  // 开启PID控制
  myPID.SetMode(AUTOMATIC);
}

// main
void main(void)
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
    // 调用PID控制器计算输出值
    myPID.Compute();
    // 使用output进行控制
    Output;
  }
}
