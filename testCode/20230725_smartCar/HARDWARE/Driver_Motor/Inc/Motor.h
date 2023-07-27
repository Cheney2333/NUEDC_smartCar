/**
 * @file Motor.h
 * @brief 适用于MG513编码点击的电机驱动头文件
 * @version 1.1
 * @date 2023/7/19
 * @note None
*/
#ifndef __MOTOR_H__
#define __MOTOR_H__

/***************************************************************************/
//include
#include "stm32f1xx_hal.h"
#include "main.h"
#include "tim.h"
#include "PID.h"    //可以和PID联动进行闭环控制
#include "MPU.h"    //可以和MPU6050联动转直角弯

/***************************************************************************/
//C++ compatible
#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************/
//define
#define motor_TIM           &htim4
#define AMotorChannel       TIM_CHANNEL_3
#define BMotorChannel       TIM_CHANNEL_4

#define LeftForwardSpeed    0.1 
#define RightForwardSpeed   0.1

#define LeftBackSpeed       -0.1
#define RightBackSpeed      -0.1

//接下来是一些基本操作定义
#define SetSpeed_Forward()  leftTargetSpeed  = LeftForwardSpeed;     \
                            rightTargetSpeed = RightForwardSpeed    
#define SetSpeed_Back()     leftTargetSpeed  = LeftBackSpeed;        \
                            rightTargetSpeed = RightBackSpeed  
#define SetSpeed_Stop()     leftTargetSpeed  = 0;                    \
                            rightTargetSpeed = 0
#define SetSpeed_Left()     leftTargetSpeed  = LeftBackSpeed;        \
                            rightTargetSpeed = RightForwardSpeed
#define SetSpeed_Right()    leftTargetSpeed  = LeftForwardSpeed;     \
                            rightTargetSpeed = RightBackSpeed  

#define GetSpeed()          GetEncoderPulse();                              \
                            leftSpeed = -CalActualSpeed(encoderPulse[0]);   \
                            rightSpeed = CalActualSpeed(encoderPulse[1])

/***************************************************************************/
//extern variable
extern short encoderPulse[2];
extern float leftTargetSpeed, rightTargetSpeed;
extern float leftSpeed, rightSpeed;

/***************************************************************************/
//function

void AMotor_Go(void);   //电机A正转
void AMotor_Back(void); //电机A反转
void AMotor_Stop(void); //电机A停止

void BMotor_Go(void);   //电机B正转
void BMotor_Back(void); //电机B反转
void BMotor_Stop(void); //电机B停止

void MotorControl(int AMotorPWM, int BMotorPWM);  //控制电机转速

void LeftBend(float currentAngle);  //左直角弯
void RightBend(float currentAngle); //右直角弯

void GetEncoderPulse(void); //读取编码器脉冲值

float CalActualSpeed(int pulse);  //计算速度

/***************************************************************************/
//C++ compatible
#ifdef __cplusplus
}
#endif

#endif

