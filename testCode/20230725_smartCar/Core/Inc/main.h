/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
  #include "Motor.h"
  #include "PID.h"
  #include "stdio.h"
  #include "string.h"
  #include "I2C_Simulate.h"
  #include "MPU.h"
  #include "inv_mpu.h"
  #include "inv_mpu_dmp_motion_driver.h"
  #include "CCD_TSL1401.h"
  #include "VL53L0X.h"
  #include "OLED.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
  extern I2C_HandleTypeDef  hi2c1;
  extern ADC_HandleTypeDef  hadc1;
  //extern uint8_t            TempStr[30];

  extern uint8_t            Uart2RxBuff[2];           //UART2单次接收数据缓冲区
  extern uint8_t            Uart2DataBuff[100];       //UART2接收数据缓冲区
  extern uint8_t            RxLine;                   //接受长度
  extern uint8_t            Uart2RxFlag;              //UART2接收标志位

  extern uint8_t            MovementPara;             //运动参数

  extern volatile uint32_t  adcBuffer[20];            // 保存ADC转换后的数值
  extern float              ADC_Value[2];             // 保存计算后的数值
  extern float              temperature;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

  void MPU6050_GetData(void);

  void MovementPara_Decode(void);

  void OLEDShow(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CCD_AO_Pin GPIO_PIN_0
#define CCD_AO_GPIO_Port GPIOC
#define CCD_SI_Pin GPIO_PIN_1
#define CCD_SI_GPIO_Port GPIOC
#define CCD_CLK_Pin GPIO_PIN_2
#define CCD_CLK_GPIO_Port GPIOC
#define AMotorEncoderA_Pin GPIO_PIN_0
#define AMotorEncoderA_GPIO_Port GPIOA
#define AMotorEncoderB_Pin GPIO_PIN_1
#define AMotorEncoderB_GPIO_Port GPIOA
#define BMotorEncoderA_Pin GPIO_PIN_6
#define BMotorEncoderA_GPIO_Port GPIOA
#define BMotorEncoderB_Pin GPIO_PIN_7
#define BMotorEncoderB_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_8
#define KEY1_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_9
#define KEY2_GPIO_Port GPIOE
#define KEY3_Pin GPIO_PIN_10
#define KEY3_GPIO_Port GPIOE
#define KEY4_Pin GPIO_PIN_11
#define KEY4_GPIO_Port GPIOE
#define TCRT_D0_Pin GPIO_PIN_12
#define TCRT_D0_GPIO_Port GPIOE
#define Buzzer_IO_Pin GPIO_PIN_13
#define Buzzer_IO_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_15
#define LED_GREEN_GPIO_Port GPIOE
#define VL53L0X_SCL_Pin GPIO_PIN_10
#define VL53L0X_SCL_GPIO_Port GPIOB
#define VL53L0X_SDA_Pin GPIO_PIN_11
#define VL53L0X_SDA_GPIO_Port GPIOB
#define TCRT_2_D0_Pin GPIO_PIN_12
#define TCRT_2_D0_GPIO_Port GPIOD
#define PWMA_Pin GPIO_PIN_14
#define PWMA_GPIO_Port GPIOD
#define PWMB_Pin GPIO_PIN_15
#define PWMB_GPIO_Port GPIOD
#define AIN1_Pin GPIO_PIN_2
#define AIN1_GPIO_Port GPIOG
#define AIN2_Pin GPIO_PIN_3
#define AIN2_GPIO_Port GPIOG
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOG
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOG
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB
#define MPU6050_SCL_Pin GPIO_PIN_8
#define MPU6050_SCL_GPIO_Port GPIOB
#define MPU6050_SDA_Pin GPIO_PIN_9
#define MPU6050_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LED_RED_ON        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 1)
#define LED_RED_OFF       HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 0);
#define LED_GREEN_ON      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1)
#define LED_GREEN_OFF     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0)
#define BUZZER_ON         HAL_GPIO_WritePin(Buzzer_IO_GPIO_Port, Buzzer_IO_Pin, 1)
#define BUZZER_OFF        HAL_GPIO_WritePin(Buzzer_IO_GPIO_Port, Buzzer_IO_Pin, 0)
#define TCRT              HAL_GPIO_ReadPin(TCRT_D0_GPIO_Port, TCRT_D0_Pin)          // 鍙崇孩澶栦紶鎰熷櫒鐢靛钩锟�?
#define TCRT_2            HAL_GPIO_ReadPin(TCRT_2_D0_GPIO_Port, TCRT_2_D0_Pin)      // 宸︾孩澶栦紶鎰熷櫒鐢靛钩锟�?
#define KEY1              HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin)
#define KEY2              HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin)
#define KEY3              HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin)
#define KEY4              HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin)

#define ADC_CHANNEL_COUNT 2                                                         // ADC通道数量
#define ADC_AVERAGE_COUNT 10                                                        // 单个通道采样值，用来取平均值
#define V25               0.76                                                      // unit: V
#define AVG_SLOPE         2.5                                                       // unit: mV/摄氏度
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
