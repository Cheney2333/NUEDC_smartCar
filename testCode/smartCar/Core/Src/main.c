/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "pid.h"
#include "oled.h"
#include "stdio.h"
#include "string.h"
#include "IIC.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "VL53L0X.h"
#include "lcd_spi_169.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PID Motor_PID[2];
PID_POSITION Motor_Position_PID[2];
PID trailMotor_PID;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int testPWM = 20;
char oledString[30];
char LCD_String[50];
int tim1Count = 0; // 中断计时
float batteryVoltage = 0.0;
float pitch, roll, yaw;    // 欧拉角
short gyrox, gyroy, gyroz; // 陀螺仪原始数据

volatile uint32_t adcBuffer[ADC_CHANNEL_COUNT * ADC_AVERAGE_COUNT]; // 保存ADC转换后的数值
float ADC_Value[ADC_CHANNEL_COUNT];                                 // 保存计算后的数值
float temperature = 0.0;                                            // 内部温度传感器

short encoderPulse[2] = {0};    // 编码器脉冲数
int totalEncoderPulse[2] = {0}; // 编码器总脉冲数
float wheelTurns[2] = {0};      // 行驶的总圈数
float targetTurn[2] = {10, -10};
float totalDistance = 0.0; // 行驶的路程

float wheelSpeed[2] = {0}; // 0为左轮，1为右轮，本工程中均如此
float targetSpeed[2] = {0.0, 0.0};

uint8_t Uart1RxBuff;         // 进入中断接收数据的数组
uint8_t Uart1DataBuff[5000]; // 保存接收到的数据的数组
int Rx1Line = 0;             // 接收到的数据长度

int mode[5] = {0};

float distance[2] = {0.0}; // unit: m
statInfo_t_VL53L0X distanceStr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  // OLED_Init(); // 初始化OLED
  // OLED_Clear();

  SPI_LCD_Init(); // SPI LCD初始化

  // Initialise the VL53L0X
  initVL53L0X(1, &hi2c2);

  // Configure the sensor for high accuracy and speed in 20 cm.
  setSignalRateLimit(200);
  setVcselPulsePeriod(VcselPeriodPreRange, 10);
  setVcselPulsePeriod(VcselPeriodFinalRange, 14);
  setMeasurementTimingBudget(300 * 1000UL);

  // MPU_Init();     // MPU6050初始化
  // mpu_dmp_init(); // dmp初始化

  HAL_TIM_Base_Start_IT(&htim1);                 // 启动定时器1中断
  HAL_UART_Receive_IT(&huart1, &Uart1RxBuff, 1); // 串口1接收中断
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2); // start encoder timer
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, ADC_CHANNEL_COUNT * ADC_AVERAGE_COUNT);
  // HAL_ADCEx_Calibration_Start(&CCD_AO_hadc);
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE); // start encoder timer to update interrupts and prevent overflow processing

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0); // initialize encoder timing and set it to 3000

  // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);

  Speed_PID_Init(&Motor_PID[0]);
  Speed_PID_Init(&Motor_PID[1]);
  Position_PID_Init(&Motor_Position_PID[0]);
  Position_PID_Init(&Motor_Position_PID[1]);
  Trail_PID_Init(&trailMotor_PID);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // OLEDShow();
    LCD_Show();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  tim1Count++;
  if (htim == &htim1) // htim1 | 100Hz | 10ms
  {
    //--------------------------PID计算------------------------------------------------
    PID_Calculate();
    MotorControl(Motor_PID[0].PWM, Motor_PID[1].PWM);

    printf("test\r\n");
    // printf("data:%.2f,%.2f,10\r\n", wheelTurns[0], wheelTurns[1]);
    //-----------------------获取电压值-------------------------------------------------
    if (tim1Count > 100) // 100 * 10 ms = 1s
    {
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      batteryVoltage = ADC_Value[0] * 5.0;
      tim1Count = 0;
    }
  }
}

void OLEDShow()
{
  sprintf(oledString, "voltage:%.1fV", batteryVoltage);
  OLED_ShowString(0, 0, (char *)oledString, 12, 0);

  sprintf(oledString, "A:%.2fm/s B:%.2fm/s      ", wheelSpeed[0], wheelSpeed[1]);
  OLED_ShowString(0, 2, (char *)oledString, 12, 0);

  sprintf(oledString, "At:%.2f Bt:%.2f    ", wheelTurns[0], wheelTurns[1]);
  OLED_ShowString(0, 4, (char *)oledString, 12, 0);
}

void MPU6050_GetData() // 获取MPU6050的数值
{
  while (mpu_dmp_get_data(&pitch, &roll, &yaw))
    ;                                                  // 必须要用while等待，才能读取成功
  MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);           // 得到陀螺仪数据
  printf("data:%.1f,%.1f,%.1f\r\n", roll, pitch, yaw); // 串口1输出采集信息
}

void PID_Calculate()
{
  GetEncoderPulse();
  wheelTurns[0] = CalNumberOfTurns(totalEncoderPulse[0]); // 获得当前行驶圈数
  wheelTurns[1] = CalNumberOfTurns(totalEncoderPulse[1]);

  targetSpeed[0] = Position_PID(targetTurn[0], wheelTurns[0], &Motor_Position_PID[0]); // 根据目标圈数计算目标速度
  targetSpeed[1] = Position_PID(targetTurn[1], wheelTurns[1], &Motor_Position_PID[1]);

  wheelSpeed[0] = CalActualSpeed(encoderPulse[0]); // 获得当前的速度值
  wheelSpeed[1] = CalActualSpeed(encoderPulse[1]);

  Speed_PID(targetSpeed[0], wheelSpeed[0], &Motor_PID[0]); // 根据目标速度和实际速度计算PID参数
  Speed_PID(targetSpeed[1], wheelSpeed[1], &Motor_PID[1]);

  distance[0] = wheelTurns[0] * PERIMETER; // 左轮路程
  distance[1] = wheelTurns[1] * PERIMETER; // 左轮路程
}

void LCD_Show()
{
  LCD_SetDirection(Direction_V);
  LCD_SetTextFont(&CH_Font20);
  LCD_SetColor(LIGHT_YELLOW);
  LCD_SetBackColor(LCD_BLACK);

  sprintf(LCD_String, "voltage:%.1fV          ", batteryVoltage);
  LCD_DisplayText(13, 10, LCD_String);
  sprintf(LCD_String, "Speed_A:%.2fm/s    ", wheelSpeed[0]);
  LCD_DisplayText(13, 40, LCD_String);
  sprintf(LCD_String, "Speed_B:%.2fm/s    ", wheelSpeed[1]);
  LCD_DisplayText(13, 70, LCD_String);
  sprintf(LCD_String, "distance_A: %.2f       ", distance[0]);
  LCD_DisplayText(13, 100, LCD_String);
  sprintf(LCD_String, "distance_B: %.2f       ", distance[1]);
  LCD_DisplayText(13, 130, LCD_String);
  sprintf(LCD_String, "Wheel_A_turns:%.2f    ", wheelTurns[0]);
  LCD_DisplayText(13, 160, LCD_String);
  sprintf(LCD_String, "Wheel_B_turns:%.2f    ", wheelTurns[1]);
  LCD_DisplayText(13, 190, LCD_String);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
