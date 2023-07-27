/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "string.h"
#include "usart.h"
#include "stdio.h"
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//uint8_t Uart2RxBuff;         // 进入中断接收数据的数组
//uint8_t Uart2DataBuff[100];   // 保存接收到的数据的数组
//int RxLine;                  // 接收到的数据长度
//int Uart2RxFlag;             // 串口2接收标志位

extern uint8_t Uart3RxBuff;         // 进入中断接收数据的数组
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2) // 判断中断源
  {
    /*
    Uart2DataBuff[RxLine] = Uart2RxBuff;      // 把每次接收到的数据保存到缓存数组
    RxLine += 1;                              // 每接收到一个数据，进入回调数据长度加1

    HAL_UART_Transmit(&huart2, (const uint8_t*)Uart2RxBuff, 1, 100);

    if (Uart2RxBuff == '|') // 接收结束标志位
    {
      oled.ssd1306.ShowString(0, 5, Uart2DataBuff, 12);

      memset(Uart2DataBuff, 0, 100*sizeof(uint8_t));    // 清空缓存数组
      RxLine = 0;                                       // 清空接收长度
    }

    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) // 溢出标志
    {
      uint32_t temp = USART2->SR;
      temp = USART2->DR;
    }

    if(RxLine == 100)   // 防止缓存数组溢出
    {
      RxLine = 0;
      memset(Uart2DataBuff, 0, 100*sizeof(uint8_t));
    }

    Uart2RxBuff = 0;
    HAL_UART_Receive_IT(&huart2, &Uart2RxBuff, 1); // 每接收一个数据，就打开一次串口中断接收，否则只会接收一个数据就停止接收
    */

    if(Uart2RxBuff[0] == '1')
      MovementPara = Uart2RxBuff[1];

    //memset(Uart2RxBuff, 0, 2*sizeof(uint8_t));
    //HAL_UART_Receive_IT(&huart2, Uart2RxBuff, 2);
  }
  
  if (huart == &huart3) // 判断中断源
  {
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE)) // 溢出标志
    {
      uint32_t temp = USART3->SR;
      temp = USART3->DR;
    }

    Uart3RxBuff = 0;
    HAL_UART_Receive_IT(&huart3, &Uart3RxBuff, 1); // 每接收一个数据，就打开一次串口中断接收，否则只会接收一个数据就停止接收
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    uint32_t sum[ADC_CHANNEL_COUNT] = {0};                           // 缓冲区求和
    float averageValue[ADC_CHANNEL_COUNT] = {0};                     // 单个通道的平均值
    for (uint16_t i = 0; i < ADC_CHANNEL_COUNT * ADC_AVERAGE_COUNT;) // 各个通道求和
    {
      sum[0] += adcBuffer[i++];
      sum[1] += adcBuffer[i++];
    }
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) // 平均值滤波
    {
      averageValue[i] = sum[i] / ADC_AVERAGE_COUNT;
    }
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
      ADC_Value[i] = averageValue[i] * 3.3 / 4096;
    }
    //temperature = (ADC_Value[2] - V25) / AVG_SLOPE + 25; // 内部温度传感器得到的温度
  }
}
/* USER CODE END 1 */
