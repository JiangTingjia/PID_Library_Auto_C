/**********************************************************************************************
*  说明：此文件PID分段控制
*  作者：蒋庭佳
*  邮箱：kyzy_duck@163.com
*  编码：GBK2312
**********************************************************************************************/
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include "stdbool.h"

// 用于记录自整定前PID控制器状态：开启还是关闭
unsigned char ATuneModeRemember = 2;
// 基本参数
double input = 80;
double output = 50;
double setpoint = 180;
unsigned long time = 0;
double kp = 2;
double ki = 0.5;
double kd = 2;

// 自整定过程中振荡输出的均值
double aTuneStartValue = 100;
// 自整定过程中振荡输出半宽，输出值会在 outputStart- aTuneStep 和 outputStart + aTuneStep 反复输出
// 如果是正向控制，那么aTuneStep设为+， 如果是反向控制 那么aTuneStep 设为负
double aTuneStep = 50;
// 输出翻转阈值，例如设定值SV，那么小于 SV - aTuneNoise, 输出outputStart + aTuneStep， 反之亦然
double aTuneNoise = 10;

unsigned int aTuneLookBack = 20;
// 自整定过程标志
bool tuning = false;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT, &time);
PID_ATune aTune(&input, &output, &time);

void Init()
{
  // 开启PID
  myPID.SetMode(AUTOMATIC);

  if (tuning)
  {
    tuning = false;
    changeAutoTune();
    tuning = true;
  }
}

void main()
{
  // 设定PID控制器基本参数
  Init();

  // 增加系统时间，可以在定时器中完成
  time++;

  while (1)
  {
    // 更新输入
    input;

    // 等待自整定结果
    if (tuning)
    {
      int val = aTune.Runtime();
      if (val != 0)
      {
        tuning = false;
      }
      if (!tuning)
      {
        // 自整定结束，更新PID控制器控制参数
        kp = aTune.GetKp();
        ki = aTune.GetKi();
        kd = aTune.GetKd();
        myPID.SetTunings(kp, ki, kd);
        AutoTuneHelper(false);
      }
    }
    else
    {
      myPID.Compute();
    }

    // 输出
    output;
  }
}

void changeAutoTune()
{
  if (!tuning)
  {
    // 现将输出中值赋值
    output = aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  {
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(bool start)
{
  if (start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}
