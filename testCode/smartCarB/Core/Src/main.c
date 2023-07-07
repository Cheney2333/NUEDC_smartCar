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
#include "i2c.h"
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PID leftMotor_PID;
PID rightMotor_PID;
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
char voltage[20];
char mpuString[10];
char speedString[22];
char CCDString[20];
char colorPostion[20];
int tim1Count = 0; // 中断计时
float batteryVoltage = 0.0;
float pitch, roll, yaw;    // 欧拉角
short gyrox, gyroy, gyroz; // 陀螺仪原始数据
float refrenceAngle = 0.0;

uint32_t CCD_Value[128]; // CCD数据数组
uint32_t Threshold = 0;  // CCD阈值

short encoderPulse[2] = {0}; // 编码器脉冲数
float leftSpeed = 0, rightSpeed = 0;
int direction = 0; // 前进方向，0为去程，1为返程，2为结束

float leftTargetSpeed = 0.10, rightTargetSpeed = 0.10;

uint8_t Uart2RxBuff;         // 进入中断接收数据的数组
uint8_t Uart2DataBuff[5000]; // 保存接收到的数据的数组
int RxLine = 0;              // 接收到的数据长度
int Uart2RxFlag = 0;         // 串口2接收标志位

int RedX = 0, RedY = 0;
int Basic_1_Status = 0, Basic_2_Status = 0;

int girdsNum = -1;      // 格子数量
int girdsNumStatus = 0; // 格子数量状态量，用于判断是否持续扫描到黑线
int backStatus = 0;     // 基础1返程标志

int ledGreenCount = 0;
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init(); // 初始化OLED
  OLED_Clear();

  HAL_TIM_Base_Start_IT(&htim1);                 // 启动定时器1中断
  HAL_UART_Receive_IT(&huart2, &Uart2RxBuff, 1); // 串口2接收中断
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2); // start encoder timer
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE); // start encoder timer to update interrupts and prevent overflow processing

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0); // initialize encoder timing and set it to 3000

  Speed_PID_Init(&leftMotor_PID);
  Speed_PID_Init(&rightMotor_PID);
  Trail_PID_Init(&trailMotor_PID);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Main_Loop();
    MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
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
  ledGreenCount++;
  tim1Count++;
  if (htim == &htim1) // htim1 100Hz 10ms
  {
    // GirdsNumber();
    GetEncoderPulse();
    leftSpeed = CalActualSpeed(encoderPulse[0]); // 获得当前的速度值
    rightSpeed = CalActualSpeed(encoderPulse[1]);

    Speed_PID(leftTargetSpeed, leftSpeed, &leftMotor_PID); // 根据目标速度和实际速度计算PID参数
    Speed_PID(rightTargetSpeed, rightSpeed, &rightMotor_PID);

    MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);

    // MotorControl(25, 25);

    // printf("data:%.2f,%.2f,%.2f\r\n", leftSpeed, rightSpeed, leftTargetSpeed);
    // printf("x = %d, y = %d\r\n", RedX, RedY);

    //-----------------------获取电压值-------------------------------------------------
    // if (tim1Count > 100) // 1S
    // {
    //   batteryVoltage = adcGetBatteryVoltage();
    //   tim1Count = 0;
    // }
    // //-----------------------绿灯闪烁2S-------------------------------------------------
    // if (ledGreenCount > 200 && direction != 2) // 2S
    // {
    //   HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin); // 绿灯闪烁
    //   ledGreenCount = 0;
    // }
    // //-----------------------基础1部分---------------------------------------------------
    // if (Basic_1_Status == 0 && Basic_2_Status == 0)
    // {
    //   //---------------------基础1去程--------------------------------------------------
    //   if (direction == 0 && backStatus == 0)
    //   {
    //     if (RedY < 225)
    //     {
    //       Trail_PID(RedX, &trailMotor_PID);
    //       leftTargetSpeed = 0.10 + trailMotor_PID.Un;
    //       rightTargetSpeed = 0.10 - trailMotor_PID.Un;
    //     }
    //     else
    //     {
    //       RedX = 60;
    //       Trail_PID(RedX, &trailMotor_PID);
    //       leftTargetSpeed = 0.10 + trailMotor_PID.Un;
    //       rightTargetSpeed = 0.10 - trailMotor_PID.Un;
    //     }
    //     if (girdsNum == 25) // 去程抵达终点
    //     {
    //       backStatus = 1;
    //       direction = 1; // 进入返程模式
    //       // girdsNum = -1; // 格子计数清零
    //     }
    //     MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
    //   }
    //   //---------------------基础1抵达终点后至返程前-----------------------------------
    //   else if (direction == 1 && backStatus == 1)
    //   {
    //     MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
    //   }
    //   //---------------------基础1返程------------------------------------------------
    //   else if (direction == 1 && backStatus == 2)
    //   {
    //     if (RedY < 225)
    //     {
    //       Trail_PID(RedX, &trailMotor_PID);
    //       leftTargetSpeed = 0.10 + trailMotor_PID.Un;
    //       rightTargetSpeed = 0.10 - trailMotor_PID.Un;
    //     }
    //     else
    //     {
    //       RedX = 260;
    //       Trail_PID(RedX, &trailMotor_PID);
    //       leftTargetSpeed = 0.10 + trailMotor_PID.Un;
    //       rightTargetSpeed = 0.10 - trailMotor_PID.Un;
    //     }
    //     if (girdsNum == 9) // T型路口直接右转
    //     {
    //       RedX = 260;
    //       Trail_PID(RedX, &trailMotor_PID);
    //       leftTargetSpeed = 0.10 + trailMotor_PID.Un;
    //       rightTargetSpeed = 0.10 - trailMotor_PID.Un;
    //     }
    //     else if (girdsNum == 25) // 返程抵达起始点1
    //     {
    //       backStatus = 3;
    //       direction = 2; // 返程结束
    //       // girdsNum = -1; // 格子计数清零
    //     }
    //     MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
    //   }
    //   //----------------------基础1结束------------------------------------------------
    //   else if (direction == 2)
    //   {
    //   }
    // }
  }
}

void Main_Loop()
{
  OLEDShow();
  // Basic_1();
}

void Basic_1()
{
  if (direction != 2)
  {
    if (direction == 1 && backStatus == 1) // 返程
    {
      leftTargetSpeed = 0;
      rightTargetSpeed = 0; // 停车
      BUZZER_ON;
      HAL_Delay(400);
      BUZZER_OFF;      // 蜂鸣示意抵达终点
      HAL_Delay(5000); // 等待5秒

      leftTargetSpeed = 0.10;
      rightTargetSpeed = -0.10; // 原地掉头，准备返程
      HAL_Delay(2800);

      backStatus = 2;
    }
  }
  else if (direction == 2 && backStatus == 3) // 停止工作
  {
    leftTargetSpeed = 0;
    rightTargetSpeed = 0;
    HAL_Delay(100);
    LED_GREEN_OFF;
    MotorControl(0, 0);
  }
}

void Basic_2()
{
}

void Buzzer() // 蜂鸣器鸣叫200ms，间隔1秒
{
  BUZZER_ON;
  HAL_Delay(200);
  BUZZER_OFF;
  HAL_Delay(1000);
}

void LED_GREEN_2S()
{
  LED_GREEN_ON;
  HAL_Delay(2000);
  LED_GREEN_OFF;
  HAL_Delay(2000);
}
void LED_RED_1S()
{
  LED_RED_ON;
  HAL_Delay(1000);
  LED_RED_OFF;
}

void GirdsNumber()
{

  if (TCRT == 0) // 扫描到黑线
  {
    girdsNumStatus++;

    if (girdsNumStatus == 5)
    {
      if (direction == 0 && backStatus == 0)      // 基础1去程
        girdsNum++;                               // 格子数量加1
      else if (direction == 1 && backStatus == 2) // 基础1返程
        girdsNum--;                               // 格子数量减1
    }
  }
  else if (TCRT == 1) // 未扫描到黑线
    girdsNumStatus = 0;
}

void OLEDShow()
{
  sprintf(voltage, "batteryVoltage:%.1fV", batteryVoltage);
  OLED_ShowString(0, 0, (char *)voltage, 12, 0);

  sprintf(speedString, "A:%.2fm/s B:%.2fm/s", leftSpeed, rightSpeed);
  OLED_ShowString(0, 2, (char *)speedString, 12, 0);

  sprintf(colorPostion, "x:%d ", RedX);
  OLED_ShowString(0, 4, (char *)colorPostion, 12, 0);
  sprintf(colorPostion, "y:%d ", RedY);
  OLED_ShowString(60, 4, (char *)colorPostion, 12, 0);

  sprintf(colorPostion, "girdNum:%d    ", girdsNum);
  OLED_ShowString(0, 6, (char *)colorPostion, 12, 0);
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