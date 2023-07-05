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
#include "IIC.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "CCD_TSL1401.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PID leftMotor_PID;
PID rightMotor_PID;
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
uint8_t g_ucUsart2ReceiveData; // 保存串口2接收的数据
char voltage[20];
char mpuString[10];
char speedString[22];
char CCDString[20];
int tim1Count = 0;
float batteryVoltage = 0.0;
float pitch, roll, yaw;    // 欧拉角
short aacx, aacy, aacz;    // 加速度传感器原始数据
short gyrox, gyroy, gyroz; // 陀螺仪原始数据
float ax, ay, az;
float temp; // 温度
float refrenceAngle = 0.0;

uint32_t CCD_Value[128]; // CCD数据数组
uint32_t Threshold = 0;  // CCD阈值

short encoderPulse[2] = {0}; // 编码器脉冲数
float leftSpeed = 0, rightSpeed = 0;

float leftTargetSpeed = 0.10, rightTargetSpeed = -0.10;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Main_Loop(void);
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

  MPU_Init();     // MPU6050初始化
  mpu_dmp_init(); // dmp初始化
  // printf("初始化成功！\r\n");

  HAL_TIM_Base_Start_IT(&htim1);                           // 启动定时器1中断
  HAL_UART_Receive_IT(&huart2, &g_ucUsart2ReceiveData, 1); // 串口2接收中断
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2); // start encoder timer
  HAL_ADCEx_Calibration_Start(&CCD_AO_hadc);
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE); // start encoder timer to update interrupts and prevent overflow processing

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0); // initialize encoder timing and set it to 3000

  PID_Init(&leftMotor_PID);
  PID_Init(&rightMotor_PID);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Main_Loop();
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  if (htim == &htim1) // htim1 100Hz 10ms
  {
    GetEncoderPulse();
    leftSpeed = CalActualSpeed(encoderPulse[0]); // 获得当前的速度值
    rightSpeed = CalActualSpeed(encoderPulse[1]);

    Speed_PID(leftTargetSpeed, leftSpeed, &leftMotor_PID); // 根据目标速度和实际速度计算PID参数
    Speed_PID(rightTargetSpeed, rightSpeed, &rightMotor_PID);

    // MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);

    // MotorControl(25, 25);

    MPU6050_GetData();

    // printf("data:%.2f,%.2f,%.2f\r\n", leftSpeed, rightSpeed, leftTargetSpeed);

    // CCD_Read(CCD_Value);
    // CCD_Data_Transform(CCD_Value);
    // Threshold = CCD_CalcuThreshold(CCD_Value);
    // CCD_Binarization(CCD_Value, Threshold);
    // CCD_CalcuPosition(CCD_Value);
    // CCD_Value_Clear(CCD_Value);

    if (tim1Count > 20)
    {
      batteryVoltage = adcGetBatteryVoltage();
      // printf("test");
      tim1Count = 0;
    }
  }
}
void MPU6050_GetData() // 获取MPU6050的数值
{
  while (mpu_dmp_get_data(&pitch, &roll, &yaw))
    ; // 必须要用while等待，才能读取成功
  // MPU_Get_Accelerometer(&aacx, &aacy, &aacz); // 得到加速度传感器数据
  // ax = (float)aacx / 16384.0;
  // ay = (float)aacy / 16384.0;
  // az = (float)aacz / 16384.0;
  MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz); // 得到陀螺仪数据
  // temp = MPU_Get_Temperature();                        // 得到温度信息
  printf("data:%.1f,%.1f,%.1f\r\n", roll, pitch, yaw); // 串口1输出采集信息
}

void Main_Loop()
{
  // OLED_ShowString(0, 0, (char *)"", 12, 0);
  sprintf(voltage, "batteryVoltage:%.1fV", batteryVoltage);
  OLED_ShowString(0, 0, (char *)voltage, 12, 0);

  sprintf(speedString, "A:%.2fm/s B:%.2fm/s", leftSpeed, rightSpeed);
  OLED_ShowString(0, 2, (char *)speedString, 12, 0);

  sprintf(CCDString, "threshold:%d", Threshold);
  OLED_ShowString(0, 4, (char *)CCDString, 12, 0);

  sprintf(CCDString, "left:%d", Position[0]);
  OLED_ShowString(0, 6, (char *)CCDString, 12, 0);
  sprintf(CCDString, "right:%d", Position[1]);
  OLED_ShowString(50, 6, (char *)CCDString, 12, 0);

  // MotorControl(0, 0);
  // refrenceAngle = pitch;
  // LeftBend(refrenceAngle);
  // MotorControl(0, 0);
  // refrenceAngle = pitch;
  // RightBend(refrenceAngle);
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

#ifdef USE_FULL_ASSERT
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
