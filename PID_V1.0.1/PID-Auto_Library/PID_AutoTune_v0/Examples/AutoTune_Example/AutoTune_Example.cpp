/**********************************************************************************************
*  ˵�������ļ�PID�ֶο���
*  ���ߣ���ͥ��
*  ���䣺kyzy_duck@163.com
*  ���룺GBK2312
**********************************************************************************************/
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include "stdbool.h"

// ���ڼ�¼������ǰPID������״̬���������ǹر�
unsigned char ATuneModeRemember = 2;
// ��������
double input = 80;
double output = 50;
double setpoint = 180;
unsigned long time = 0;
double kp = 2;
double ki = 0.5;
double kd = 2;

// ������������������ľ�ֵ
double aTuneStartValue = 100;
// �����������������������ֵ���� outputStart- aTuneStep �� outputStart + aTuneStep �������
// �����������ƣ���ôaTuneStep��Ϊ+�� ����Ƿ������ ��ôaTuneStep ��Ϊ��
double aTuneStep = 50;
// �����ת��ֵ�������趨ֵSV����ôС�� SV - aTuneNoise, ���outputStart + aTuneStep�� ��֮��Ȼ
double aTuneNoise = 10;

unsigned int aTuneLookBack = 20;
// ���������̱�־
bool tuning = false;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT, &time);
PID_ATune aTune(&input, &output, &time);

void Init()
{
  // ����PID
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
  // �趨PID��������������
  Init();

  // ����ϵͳʱ�䣬�����ڶ�ʱ�������
  time++;

  while (1)
  {
    // ��������
    input;

    // �ȴ����������
    if (tuning)
    {
      int val = aTune.Runtime();
      if (val != 0)
      {
        tuning = false;
      }
      if (!tuning)
      {
        // ����������������PID���������Ʋ���
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

    // ���
    output;
  }
}

void changeAutoTune()
{
  if (!tuning)
  {
    // �ֽ������ֵ��ֵ
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
