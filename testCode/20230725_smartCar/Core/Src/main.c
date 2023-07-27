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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PID                 leftMotor_PID;
PID                 rightMotor_PID;
PID                 trailMotor_PID;

uint8_t             TempStr[30];                                                  // OLED显示缓存

int                 tim1Count           = 0;                                      // 中断计时

float               batteryVoltage      = 0.0;                                    // 电池电压

short               encoderPulse[2]     = {0};                                    // 编码器脉冲数
float               leftSpeed           = 0;                                      // 左轮速度
float               rightSpeed          = 0;                                      // 右轮速度

float               leftTargetSpeed     = 0;                                      // 左轮目标速度
float               rightTargetSpeed    = 0;                                      // 右轮目标速度

uint8_t             Uart2RxBuff[2];                                               // 进入中断接收数据的数组
uint8_t             Uart2DataBuff[100];                                           // 保存接收到的数据的数组
uint8_t             RxLine              = 0;                                      // 接收到的数据长度
uint8_t             Uart2RxFlag         = 0;                                      // 串口2接收标志位

uint8_t             MovementPara        = 0;                                      // 运动参数

uint8_t             Uart3RxBuff;                                                  // 进入中断接收数据的数组
int                 Uart3RxFlag         = 0;                                      // 串口3接收标志位

uint16_t            distance            = 0;                                      // unit: mm
statInfo_t_VL53L0X  distanceStr;

volatile uint32_t   adcBuffer[ADC_CHANNEL_COUNT * ADC_AVERAGE_COUNT];             // 保存ADC转换后的数值
float               ADC_Value[ADC_CHANNEL_COUNT];                                 // 保存计算后的数值
float               temperature = 0.0;                                            // 内部温度传感器
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
  /* USER CODE BEGIN 2 */
  SSD1306_Init(&hi2c1); // 初始化OLED
  oled.ssd1306.Clear();

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0); // 设置SysTick中断优先级，防止卡死

  // Initialise the VL53L0X
  //initVL53L0X(1, &hi2c2);

  // Configure the sensor for high accuracy and speed in 20 cm.
  //setSignalRateLimit(200);
  //setVcselPulsePeriod(VcselPeriodPreRange, 10);
  //setVcselPulsePeriod(VcselPeriodFinalRange, 14);
  //setMeasurementTimingBudget(300 * 1000UL);

  MPU6050_Init();     // MPU6050初始化
  mpu_dmp_init(); // dmp初始化

  HAL_TIM_Base_Start_IT(&htim1);                 // 启动定时器1中断
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
  
  // HAL_ADCEx_Calibration_Start(&CCD_AO_hadc);

  Speed_PID_Init(&leftMotor_PID);
  Speed_PID_Init(&rightMotor_PID);
  Trail_PID_Init(&trailMotor_PID);

  //HAL_UART_Receive_IT(&huart2, &Uart2RxBuff, 1); // 串口2接收中断
  HAL_UART_Receive_IT(&huart2, Uart2RxBuff, 2); // 串口2接收中断
  memset(Uart2DataBuff, 0, 100*sizeof(uint8_t));

  //HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, ADC_CHANNEL_COUNT * ADC_AVERAGE_COUNT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    OLEDShow(); // OLED显示
    MovementPara_Decode(); // 运动参数解码
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
  if (htim == &htim1) // htim1 100Hz 10ms
  {
    GetSpeed(); // 读取编码器速度，存储在leftSpeed和rightSpeed中

    Speed_PID(leftTargetSpeed, leftSpeed, &leftMotor_PID); // 根据目标速度和实际速度计算PID参数
    Speed_PID(rightTargetSpeed, rightSpeed, &rightMotor_PID);

    //MotorControl(25, 25);

    MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM); // 电机控制

    MPU6050_GetData();  //读取MPU6050数据

    if (tim1Count > 100)
    {
      batteryVoltage = adcGetBatteryVoltage();  //获取电压值
      tim1Count = 0;
    }
  }
}

void MovementPara_Decode()
{
  if(Uart2RxBuff[0] != 0 && Uart2RxBuff[1] != 0)
  {
    if(MovementPara == '0')  //停下
    {
      leftTargetSpeed = 0;
      rightTargetSpeed = 0;
      printf("stop");
    }
    else if(MovementPara == '1') //前进
    {
      leftTargetSpeed = LeftForwardSpeed;
      rightTargetSpeed = RightForwardSpeed;
      printf("forward");
    }
    else if(MovementPara == '2') //左转
    {
      leftTargetSpeed = LeftBackSpeed;
      rightTargetSpeed = RightForwardSpeed;
      printf("left");
    }
    else if(MovementPara == '3') //后退
    {
      leftTargetSpeed = LeftBackSpeed;
      rightTargetSpeed = RightBackSpeed;
      printf("back");
    }
    else if(MovementPara == '4') //右转
    {
      leftTargetSpeed = LeftForwardSpeed;
      rightTargetSpeed = RightBackSpeed;
      printf("right");
    }

    printf("%s\r\n", Uart2RxBuff);
    memset(Uart2RxBuff, 0, 2*sizeof(uint8_t)); // 清空接收缓存
    HAL_UART_Receive_IT(&huart2, Uart2RxBuff, 2); // 串口2接收中断
  }
  else
  {
    memset(Uart2RxBuff, 0, 2*sizeof(uint8_t)); // 清空接收缓存
    HAL_UART_Receive_IT(&huart2, Uart2RxBuff, 2); // 串口2接收中断
  }
}

void OLEDShow()
{

  sprintf((char *)TempStr, "voltage:%.1fV", batteryVoltage);
  oled.ssd1306.ShowString(0, 0, TempStr, 12);

  sprintf((char *)TempStr, "A%.2f B%.2f", leftSpeed, rightSpeed);
  oled.ssd1306.TempStrCheck(TempStr, 13);
  oled.ssd1306.ShowString(0, 1, TempStr, 12);
  
  memset(TempStr, 0, sizeof(TempStr));

  /*
  sprintf((char *)TempStr, "              ");
  oled.ssd1306.ShowString(0, 2, TempStr, 12);

  sprintf((char *)TempStr, "P:%.2f gx:%d", (mpu.mpu6050.pitch), (mpu.mpu6050.gyrox));
  oled.ssd1306.ShowString(0, 2, TempStr, 12);

  sprintf((char *)TempStr, "              ");
  oled.ssd1306.ShowString(0, 3, TempStr, 12);

  sprintf((char *)TempStr, "R:%.2f gy:%d", (mpu.mpu6050.roll), (mpu.mpu6050.gyroy));
  oled.ssd1306.ShowString(0, 3, TempStr, 12);

  sprintf((char *)TempStr, "              ");
  oled.ssd1306.ShowString(0, 4, TempStr, 12);

  sprintf((char *)TempStr, "Y:%.2f gz:%d", (mpu.mpu6050.yaw), (mpu.mpu6050.gyroz));
  oled.ssd1306.ShowString(0, 4, TempStr, 12);
  */
  
  sprintf((char *)TempStr, "P%.2f", (mpu.mpu6050.pitch));
  oled.ssd1306.TempStrCheck(TempStr, 8);
  oled.ssd1306.ShowString(0, 2, TempStr, 12);
  memset(TempStr, 0, sizeof(TempStr));

  sprintf((char *)TempStr, "R%.2f", (mpu.mpu6050.roll));
  oled.ssd1306.TempStrCheck(TempStr, 8);
  oled.ssd1306.ShowString(0, 3, TempStr, 12);
  memset(TempStr, 0, sizeof(TempStr));

  sprintf((char *)TempStr, "Y%.2f ", (mpu.mpu6050.yaw));
  oled.ssd1306.TempStrCheck(TempStr, 8);
  oled.ssd1306.ShowString(0, 4, TempStr, 12);
  memset(TempStr, 0, sizeof(TempStr));

  sprintf((char *)TempStr, "gx%d", (mpu.mpu6050.gyrox));
  oled.ssd1306.TempStrCheck(TempStr, 7);
  oled.ssd1306.ShowString(9*SSD1306_EnXLen, 2, TempStr, 12);
  memset(TempStr, 0, sizeof(TempStr));

  sprintf((char *)TempStr, "gy%d", (mpu.mpu6050.gyroy));
  oled.ssd1306.TempStrCheck(TempStr, 7);
  oled.ssd1306.ShowString(9*SSD1306_EnXLen, 3, TempStr, 12);
  memset(TempStr, 0, sizeof(TempStr));

  sprintf((char *)TempStr, "gz%d", (mpu.mpu6050.gyroz));
  oled.ssd1306.TempStrCheck(TempStr, 7);
  oled.ssd1306.ShowString(9*SSD1306_EnXLen, 4, TempStr, 12);
  memset(TempStr, 0, sizeof(TempStr));

  //sprintf((char *)TempStr, "%ds", (int)(uwTick*0.001));
  //oled.ssd1306.ShowString(12*SSD1306_EnXLen, 1, TempStr, 12);
  //memset(TempStr, 0, sizeof(TempStr));

  sprintf((char *)TempStr, "Move%c", MovementPara);
  oled.ssd1306.ShowString(0, 5, TempStr, 12);
  memset(TempStr, 0, sizeof(TempStr));

  sprintf((char *)TempStr, "%s", Uart2RxBuff);
  oled.ssd1306.ShowString(0, 6, TempStr, 12);
}

void MPU6050_GetData() // 获取MPU6050的数值
{
  while (mpu_dmp_get_data(&(mpu.mpu6050.pitch), &(mpu.mpu6050.roll), &(mpu.mpu6050.yaw)));        // 必须要用while等待，才能读取成功
  mpu.mpu6050.Get_Gyroscope(&(mpu.mpu6050.gyrox), &(mpu.mpu6050.gyroy), &(mpu.mpu6050.gyroz));    // 得到陀螺仪数据
  //printf("data:%.1f,%.1f,%.1f\r\n", roll, &(mpu.mpu6050.pitch), yaw);  // 串口1输出采集信息
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
