/*
 * @file    CCD.h
 * @brief   CCD 头文件
 * @details 包含所有CCD的预定义和函数声明
 * @version V3.1
 * @date    2023/7/11
 * @note    记得把这句定义加到main.h里面去
 *          #include "CCD.h"
 *          extern I2C_HandleTypeDef hi2c1;
 * @note    记得把这句定义加到main.c里面去
 *          HAL_ADCEx_Calibration_Start(&CCD_AO_hadc);  //校准ADC，复制到main函数里
 */

#ifndef __CCD_H
#define __CCD_H

/************************************************************************/
//Include
#include "stm32f1xx_hal.h"
#include "main.h"
#include "CCD_TSL1401.h"

/*************************************************************************/
//struct
extern struct CCD
{
    CCD_TSL1401_Handler tsl1401;
} ccd;

#endif /* __CCD_TSL1401_H */
