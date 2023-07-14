/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : remoteControl.c
 * @brief          : Main program body
 * @date           : 2023/07/13
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 Cheney.
 * All rights reserved.
 *
 ******************************************************************************
 */
#include "remoteControl.h"

float TargetSpeed[2] = {0};                // 0为左轮速度，1为右轮速度
float refrenceVR[2] = {0};                 // 参考电压，即二者速度为0是的基准电压
extern float ADC_Value[ADC_CHANNEL_COUNT]; // 三个通道的数值

void RefrenceVR_Init(void) /// 参考电压初始化
{
    refrenceVR[0] = ADC_Value[0]; // VRx
    refrenceVR[1] = ADC_Value[1]; // VRy
}

void Speed_Calculate()
{
    float speedStraight = 0.0;             // 前后方向的速度分量
    float speedAroud = 0.0;                // 左右方向的速度分量
    if (refrenceVR[0] - ADC_Value[0] == 0) // 摇杆无y分量
    {
        speedStraight = 0;
    }
    else if (refrenceVR[0] - ADC_Value[0] > 0) // 摇杆偏上
    {
        speedStraight = (refrenceVR[0] - ADC_Value[0]) * 1.5 / refrenceVR[0];
    }
    else if (refrenceVR[0] - ADC_Value[0] < 0) // 摇杆偏下
    {
        speedStraight = (refrenceVR[0] - ADC_Value[0]) * 1.5 / (3.3 - refrenceVR[0]);
    }

    if (refrenceVR[1] - ADC_Value[1] == 0) // 摇杆无x分量
    {
        speedAroud = 0;
    }
    else if (refrenceVR[1] - ADC_Value[1] > 0) // 摇杆偏右
    {
        speedAroud = (refrenceVR[1] - ADC_Value[1]) * 0.2 / refrenceVR[1];
    }
    else if (refrenceVR[1] - ADC_Value[1] < 0) // 摇杆偏左
    {
        speedAroud = (refrenceVR[1] - ADC_Value[1]) * 0.2 / (3.3 - refrenceVR[1]);
    }

    TargetSpeed[0] = speedStraight + speedAroud; // 左轮速度
    TargetSpeed[1] = speedStraight - speedAroud; // 右轮速度
}
