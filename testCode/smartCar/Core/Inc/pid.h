/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    motor.h
 * @brief   This file contains all the function prototypes for
 *          the pid.c file
 ******************************************************************************
 * @attention
 ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* USER CODE BEGIN Includes */
#include "main.h"
  /* USER CODE END Includes */

  /* USER CODE BEGIN Private defines */

  /*
   *  定义PID的结构体，结构体内存储PID参数、误差、限幅值以及输出值
   */
  typedef struct
  {
    // 相关速度PID参数
    float Velcity_Kp;
    float Velcity_Ki;
    float Velcity_Kd;
    float Ur; // 限幅值

    int PID_is_Enable; // PID使能
    int Un;            // 期望输出值
    int En_1;          // 上一次的误差值
    int En_2;          // 上上次的误差值
    int PWM;           // 输出PWM值

  } PID;
  /* USER CODE END Private defines */

  /* USER CODE BEGIN Prototypes */
  void PID_Init(PID *p);                                         // PID值初始化
  void Speed_PID(float targetSpeed, float currentSpeed, PID *p); // 计算PID速度
  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__PID_H__ */
