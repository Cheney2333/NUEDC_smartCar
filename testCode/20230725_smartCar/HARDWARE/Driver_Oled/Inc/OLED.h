/*
 * @file    OLED.h
 * @brief   OLED header file
 * @details This file including all define and function prototypes
 *         of OLED_xxx.c
 * @version V3.1
 * @date    2023/7/11
 * @note    记得在main.h里添加：#include "OLED.h" 和 extern I2C_HandleTypeDef hi2c1;
 */
#ifndef __OLED_H
#define __OLED_H

/***********************************************************************/
//include
#include "stm32f1xx_hal.h"
#include "main.h"
#include "codetab.h"
#include "OLED_SSD1306.h"

/***********************************************************************/
//struct
extern struct OLED
{
    OLED_SSD1306_Handler ssd1306;
} oled;

/***********************************************************************/
//variable




#endif
