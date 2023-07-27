/**
 * @file PID.h
 * @brief PID头文件
 * @version 1.0
 * @date 2023/7/17
 * @note None
*/
#ifndef __PID_H__
#define __PID_H__

/***************************************************************************/
//C++ compatible
#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************/
//include
#include "main.h"

/***************************************************************************/
//struct
typedef struct
{//定义PID的结构体，结构体内存储PID参数、误差、限幅值以及输出值
  
  // 相关速度PID参数
  float Kp;
  float Ki;
  float Kd;
  float Ur; // 限幅值

  int PID_is_Enable; // PID使能
  float Un;          // 期望输出值
  float En_1;        // 上一次的误差值
  float En_2;        // 上上次的误差值
  int PWM;           // 输出PWM值

} PID;

/***************************************************************************/
//extern variable
extern PID leftMotor_PID, rightMotor_PID;

/***************************************************************************/
//function

void Speed_PID_Init(PID *p); 
void Speed_PID(float targetSpeed, float currentSpeed, PID *p);
void Trail_PID_Init(PID *p);
void Trail_PID(int currentX, PID *p);


/***************************************************************************/
//C++ compatible
#ifdef __cplusplus
}
#endif

#endif /*__PID_H__ */
