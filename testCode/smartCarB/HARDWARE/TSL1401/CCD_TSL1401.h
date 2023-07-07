/*
 * @file    CCD_TSL1401.h
 * @brief   CCD_TSL1401 header file
 * @details This file including all define and function prototypes
 *         of CCD_TSL1401.c
 * @version V2.0
 * @date    2023/7/4
 * @note    记得把这句定义加到main.h里面去
 *          #include "CCD_TSL1401.h"
 *          extern I2C_HandleTypeDef hi2c1;
 * @note    记得把这句定义加到main.c里面去
 *          HAL_ADCEx_Calibration_Start(&CCD_AO_hadc);  //校准ADC，复制到main函数里
 *          uint32_t CCD_Value[128];               //CCD数据数组，复制到main函数外作为全局变量
 *          uint32_t Threshold = 0;                //阈值，复制到main函数里作为全局变量
 */

#ifndef __CCD_TSL1401_H
#define __CCD_TSL1401_H

/************************************************************************/
//Include
#include "stm32f1xx_hal.h"
#include "main.h"

/************************************************************************/
//Private define
#define CCD_AO_GPIO_Port    GPIOC
#define CCD_AO_Pin          GPIO_PIN_0
#define CCD_CLK_Pin         GPIO_PIN_2
#define CCD_CLK_GPIO_Port   GPIOC
#define CCD_SI_Pin          GPIO_PIN_1
#define CCD_SI_GPIO_Port    GPIOC

//以下定义请复制到main.h中
#define CCD_AO_CHANNEL      ADC_CHANNEL_0
#define CCD_AO_hadc         hadc1           //PC0 ADC1_IN10

#define CCD_CLK_HIGH()      HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET)
#define CCD_CLK_LOW()       HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_RESET)

#define CCD_SI_HIGH()       HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_SET)
#define CCD_SI_LOW()        HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET)

#define CCD_DELAY(us)       Delay_us(us)    //Software delay

/*************************************************************************/
//Private variable
extern ADC_HandleTypeDef CCD_AO_hadc;

extern uint8_t Position[3];
   /* Position[uint8_t *]: A pointer to store position of black line
    *   Position[0]: Left Boundry of Black Line
    *   Position[1]: Right Boundry of Black Line
    *   Position[2]: Express the color when Position[0] == Position[1] == 0
    *               1: Black
    *               0: White
    */
/*以下请复制到main.c中
uint32_t CCD_Value[128];
uint32_t Threshold = 0;
uint8_t str[20];
uint8_t CCD_Value_Exist = 0;
*/

/*************************************************************************/
//Private function
void Delay_us(uint8_t us);
void CCD_ADC_Init(void);
void CCD_StartRead(void);
void CCD_Read(uint32_t *CCD_Value);
void CCD_Data_Transform(uint32_t *CCD_Value);  
uint32_t CCD_CalcuThreshold(uint32_t *CCD_Value);
void CCD_Binarization(uint32_t *CCD_Value, uint32_t Threshold);
void CCD_CalcuPosition(uint32_t *CCD_Value);
void CCD_Value_Clear(uint32_t *CCD_Value);

#endif /* __CCD_TSL1401_H */
