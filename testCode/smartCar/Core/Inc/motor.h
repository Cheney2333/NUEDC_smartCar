/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    motor.h
 * @brief   This file contains all the function prototypes for
 *          the motor.c file
 ******************************************************************************
 * @attention
 ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

// timer and channel definition
#define motor_TIM &htim4
#define AMotorChannel TIM_CHANNEL_3
#define BMotorChannel TIM_CHANNEL_4

#define ENCODER_RESOLUTION 13    /*编码器一圈的物理脉冲数*/
#define ENCODER_MULTIPLE 4       /*编码器倍频，通过定时器的编码器模式设置*/
#define MOTOR_REDUCTION_RATIO 30 /*电机的减速比*/

  void AMotor_Go(void);
  void AMotor_Back(void);
  void AMotor_Stop(void);

  void BMotor_Go(void);
  void BMotor_Back(void);
  void BMotor_Stop(void);

  void MotorControl(int AMotorPWM, int BMotorPWM);
  void GetEncoderPulse(void);
  float CalActualSpeed(int pulse);

#ifdef __cplusplus
}
#endif
#endif /*__GPIO_H__ */
