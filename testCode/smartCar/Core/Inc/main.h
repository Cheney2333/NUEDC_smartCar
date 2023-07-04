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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void MPU6050_GetData(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CCD_AO_Pin GPIO_PIN_0
#define CCD_AO_GPIO_Port GPIOC
#define CCD_SI_Pin GPIO_PIN_1
#define CCD_SI_GPIO_Port GPIOC
#define CCD_CLK_Pin GPIO_PIN_2
#define CCD_CLK_GPIO_Port GPIOC
#define BMotorEncoderA_Pin GPIO_PIN_6
#define BMotorEncoderA_GPIO_Port GPIOA
#define BMotorEncoderB_Pin GPIO_PIN_7
#define BMotorEncoderB_GPIO_Port GPIOA
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

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
