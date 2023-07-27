/*
 * @file    MPU.h
 * @brief   MPU 头文件
 * @details 包含MPU的所有函数声明和宏定义
 * @version V1.0
 * @date    2023/7/14
 * @note    None
 */
#ifndef __MPU_H
#define __MPU_H

/*************************************************************************************************/
//include
#include "stm32f1xx_hal.h"
#include "MPU_6050.h"

/*************************************************************************************************/
//struct
extern struct MPU
{
    MPU_6050_Handler mpu6050;
} mpu;


#endif
