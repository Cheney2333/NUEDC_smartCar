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
#include "VL53L0X.h"
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
char colorPostion[30];
int tim1Count = 0; // 中断计时
float batteryVoltage = 0.0;
float pitch, roll, yaw;    // 欧拉角
short gyrox, gyroy, gyroz; // 陀螺仪原始数据
float refrenceAngle = 0.0;

uint32_t CCD_Value[128]; // CCD数据数组
uint32_t Threshold = 0;  // CCD阈值

short encoderPulse[2] = {0}; // 编码器脉冲数
float leftSpeed = 0, rightSpeed = 0;
int direction = 0; // 前进方向，0为去程，1为返程，2为结束；3为基础2去程，4为基础2返程

float leftTargetSpeed = 0.10, rightTargetSpeed = 0.10;

uint8_t Uart2RxBuff;         // 进入中断接收数据的数组
uint8_t Uart2DataBuff[5000]; // 保存接收到的数据的数组
int RxLine = 0;              // 接收到的数据长度
int Uart2RxFlag = 0;         // 串口2接收标志位

uint8_t Uart3RxBuff; // 进入中断接收数据的数组
int Uart3RxFlag = 0; // 串口3接收标志位

int RedX = 0, RedY = 0;
int Basic_1_Status = 0, Basic_2_Status = 0;

int girdsNum = -1;      // 格子数量
int girdsNumStatus = 0; // 格子数量状态量，用于判断是否持续扫描到黑线
int backStatus = 0;     // 基础1返程标志

int ledGreenCount = 0;
int ledRedCount = 0;
int mode[5] = {0};

int Triangle[26] = {0}, Square[26] = {0}, Circle[26] = {0};

int trianglePosition[2] = {0}, squarePosition[2] = {0}, circlePosition[2] = {0}; // 三个形状的位置,初始均为0

uint16_t distance = 0; // unit: mm
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
  OLED_Init(); // 初始化OLED
  OLED_Clear();

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
  HAL_UART_Receive_IT(&huart2, &Uart2RxBuff, 1); // 串口2接收中断
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2); // start encoder timer
  // HAL_ADCEx_Calibration_Start(&CCD_AO_hadc);
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
  ledGreenCount++;
  ledRedCount++;
  tim1Count++;
  if (htim == &htim1) // htim1 100Hz 10ms
  {
    // uint16_t distance is the distance in millimeters.
    // statInfo_t_VL53L0X distanceStr is the statistics read from the sensor.
    GetKeyStatus();
    GirdsNumber();
    GetEncoderPulse();
    leftSpeed = CalActualSpeed(encoderPulse[0]); // 获得当前的速度值
    rightSpeed = CalActualSpeed(encoderPulse[1]);

    Speed_PID(leftTargetSpeed, leftSpeed, &leftMotor_PID); // 根据目标速度和实际速度计算PID参数
    Speed_PID(rightTargetSpeed, rightSpeed, &rightMotor_PID);

    // MotorControl(25, 25);

    // printf("data:%.2f,%.2f,%.2f\r\n", leftSpeed, rightSpeed, leftTargetSpeed);
    // printf("x = %d, y = %d\r\n", RedX, RedY);

    //-----------------------获取电压值-------------------------------------------------
    if (tim1Count > 100) // 1S
    {
      batteryVoltage = adcGetBatteryVoltage();
      tim1Count = 0;
    }
    //-----------------------绿灯闪烁2S-------------------------------------------------
    if (ledGreenCount > 200 && direction < 2) // 2S
    {
      HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin); // 绿灯闪烁
      ledGreenCount = 0;
    }
    if (ledRedCount > 100 && direction == 4) // 紧急返程过程中，红灯闪烁、蜂鸣器鸣叫，间隔1秒
    {
      HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
      HAL_GPIO_TogglePin(Buzzer_IO_GPIO_Port, Buzzer_IO_Pin);
      ledRedCount = 0;
    }

    //-----------------------基础1部分---------------------------------------------------
    if (Basic_1_Status == 0 && Basic_2_Status == 0 && mode[0] == 1)
    {
      //---------------------基础1去程--------------------------------------------------
      if (direction == 0 && backStatus == 0)
      {
        if (RedY < 220)
        {
          Trail_PID(RedX, &trailMotor_PID);
          leftTargetSpeed = 0.10 + trailMotor_PID.Un;
          rightTargetSpeed = 0.10 - trailMotor_PID.Un;
        }
        else
        {
          RedX = 60;
          Trail_PID(RedX, &trailMotor_PID);
          leftTargetSpeed = 0.10 + trailMotor_PID.Un;
          rightTargetSpeed = 0.10 - trailMotor_PID.Un;
        }
        if (girdsNum == 25) // 去程抵达终点
        {
          backStatus = 1;
          direction = 1; // 进入返程模式
          // girdsNum = -1; // 格子计数清零
        }
        MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
      }
      //---------------------基础1抵达终点后至返程前-----------------------------------
      else if (direction == 1 && backStatus == 1)
      {
        MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
      }
      //---------------------基础1返程------------------------------------------------
      else if (direction == 1 && backStatus == 2)
      {
        if (girdsNum != 9 && direction == 1)
        {
          if (RedY < 220)
          {
            Trail_PID(RedX, &trailMotor_PID);
            leftTargetSpeed = 0.10 + trailMotor_PID.Un;
            rightTargetSpeed = 0.10 - trailMotor_PID.Un;
          }
          else
          {
            RedX = 260;
            Trail_PID(RedX, &trailMotor_PID);
            leftTargetSpeed = 0.10 + trailMotor_PID.Un;
            rightTargetSpeed = 0.10 - trailMotor_PID.Un;
          }
        }
        if (girdsNum == 1 && backStatus == 2) // 返程抵达1#方格
        {
          backStatus = 3;
          direction = 2; // 返程结束
          // girdsNum = -1; // 格子计数清零
        }
        MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
      }
      //----------------------基础1结束------------------------------------------------
      else if (direction == 2 && backStatus == 3)
      {
        MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
      }
    }
    //-------------------------基础2开始------------------------------------------------
    if (mode[1] == 1)
    {
      //-----------------------基础2去程-----------------------------------------------
      if (girdsNum < 11 && direction == 0)
      {
        if (RedY < 220)
        {
          Trail_PID(RedX, &trailMotor_PID);
          leftTargetSpeed = 0.10 + trailMotor_PID.Un;
          rightTargetSpeed = 0.10 - trailMotor_PID.Un;
        }
        else
        {
          RedX = 60;
          Trail_PID(RedX, &trailMotor_PID);
          leftTargetSpeed = 0.10 + trailMotor_PID.Un;
          rightTargetSpeed = 0.10 - trailMotor_PID.Un;
        }
        if (girdsNum == 10) // 去程抵达10#方格
        {
          backStatus = 5;
          direction = 4; // 进入返程模式
        }
        MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
      }
      if (direction == 4 && backStatus == 5) // 原地掉头
      {
        MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
      }
      //-----------------------------基础2返程----------------------------------------------
      if (direction == 4 && backStatus == 6) // 开始紧急返程
      {
        if (girdsNum != 9)
        {
          trailMotor_PID.Ur = 0.15;
          if (RedY < 220)
          {
            Trail_PID(RedX, &trailMotor_PID);
            leftTargetSpeed = 0.15 + trailMotor_PID.Un;
            rightTargetSpeed = 0.15 - trailMotor_PID.Un;
          }
          else
          {
            RedX = 250;
            Trail_PID(RedX, &trailMotor_PID);
            leftTargetSpeed = 0.15 + trailMotor_PID.Un;
            rightTargetSpeed = 0.15 - trailMotor_PID.Un;
          }
        }
        if (girdsNum == 1) // 返程抵达1#方格
        {
          backStatus = 7; // 基础2紧急返程抵达起始点1
        }
        MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
      }
    }
    //-------------------------------------发挥1部分-----------------------------------
    if (mode[2] == 1)
    {
      //---------------------发挥1去程--------------------------------------------------
      if (direction == 0 && backStatus == 0)
      {
        if (RedY < 220)
        {
          Trail_PID(RedX, &trailMotor_PID);
          leftTargetSpeed = 0.10 + trailMotor_PID.Un;
          rightTargetSpeed = 0.10 - trailMotor_PID.Un;
        }
        else
        {
          RedX = 60;
          Trail_PID(RedX, &trailMotor_PID);
          leftTargetSpeed = 0.10 + trailMotor_PID.Un;
          rightTargetSpeed = 0.10 - trailMotor_PID.Un;
        }
        if (girdsNum == 25) // 去程抵达终点
        {
          backStatus = 1;
          direction = 1; // 进入返程模式
          // girdsNum = -1; // 格子计数清零
        }
        MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
      }
      //---------------------发挥1抵达终点后至返程前-----------------------------------
      else if (direction == 1 && backStatus == 1)
      {
        MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
      }
      //---------------------发挥1返程------------------------------------------------
      else if (direction == 1 && backStatus == 2)
      {
        if (girdsNum != 9 && direction == 1)
        {
          if (RedY < 220)
          {
            Trail_PID(RedX, &trailMotor_PID);
            leftTargetSpeed = 0.10 + trailMotor_PID.Un;
            rightTargetSpeed = 0.10 - trailMotor_PID.Un;
          }
          else
          {
            RedX = 260;
            Trail_PID(RedX, &trailMotor_PID);
            leftTargetSpeed = 0.10 + trailMotor_PID.Un;
            rightTargetSpeed = 0.10 - trailMotor_PID.Un;
          }
        }
        if (girdsNum == 1 && backStatus == 2) // 返程抵达1#方格
        {
          backStatus = 3;
          direction = 2; // 返程结束
          // girdsNum = -1; // 格子计数清零
        }
        MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
      }
      //----------------------发挥1结束------------------------------------------------
      else if (direction == 2 && backStatus == 3)
      {
        MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
      }
    }
  }
}

void Main_Loop()
{
  OLEDShow();
  distance = readRangeSingleMillimeters(&distanceStr);
  if (mode[0] == 1)
    Basic_1();
  else if (mode[1] == 1)
    Basic_2();
  else if (mode[2] == 1)
    Expand_1();
  else if (mode[3] == 1)
    Expand_2();
}

void Basic_1()
{
  if (direction != 2)
  {
    if (direction == 1 && backStatus == 1) // 起始返程
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
    if (direction == 1 && backStatus == 2 && girdsNum == 9)
    {
      leftTargetSpeed = 0;
      rightTargetSpeed = 0; // 停车
      MotorControl(0, 0);
      // HAL_Delay(200);
      leftTargetSpeed = 0.10;
      rightTargetSpeed = -0.10; // 原地转右直角弯
      HAL_Delay(1400);
      leftTargetSpeed = 0.10;
      rightTargetSpeed = 0.10; // 恢复直线行驶
      HAL_Delay(2500);
    }
  }
  if (direction == 2 && backStatus == 3 && girdsNum == 0) // 停止工作
  {
    backStatus = 4;
    leftTargetSpeed = 0;
    rightTargetSpeed = 0;
    HAL_Delay(400);
    LED_GREEN_OFF;
    MotorControl(0, 0);
  }
}

void Basic_2()
{
  if (direction == 4 && backStatus == 5)
  {
    leftTargetSpeed = 0;
    rightTargetSpeed = 0;
    HAL_Delay(200);
    MotorControl(0, 0); // 停车

    leftTargetSpeed = 0.10;
    rightTargetSpeed = -0.10;
    HAL_Delay(2900); // 原地掉头，准备返程

    leftSpeed = 0.10;
    rightSpeed = 0.10;
    backStatus = 6;
    HAL_Delay(2100); // 走到9#方格

    leftTargetSpeed = 0;
    rightTargetSpeed = 0; // 停车
    HAL_Delay(200);
    MotorControl(0, 0);
  }
  if (direction == 4 && backStatus == 6 && girdsNum == 9) // 紧急返程中T型路口右转弯
  {
    leftTargetSpeed = 0;
    rightTargetSpeed = 0; // 停车
    HAL_Delay(200);
    MotorControl(0, 0);

    leftTargetSpeed = 0.10;
    rightTargetSpeed = -0.10; // 原地转右直角弯
    HAL_Delay(1400);
    leftTargetSpeed = 0.15;
    rightTargetSpeed = 0.15; // 恢复直线行驶
    HAL_Delay(2000);
  }
  if (direction == 4 && backStatus == 7 && girdsNum == 0)
  {
    backStatus = 8;
    leftTargetSpeed = 0;
    rightTargetSpeed = 0;
    HAL_Delay(100);
    MotorControl(0, 0);
    LED_GREEN_OFF;
    LED_RED_OFF;
    BUZZER_OFF;
    direction = 5;
  }
}

void Expand_1()
{
  if (Uart3RxFlag == 0)
  {
    HAL_UART_Receive_IT(&huart3, &Uart3RxBuff, 1); // 打开串口3接收中断
    Uart3RxFlag = 1;
  }
  if (direction != 2)
  {
    if (direction == 1 && backStatus == 1) // 起始返程
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
    if (direction == 1 && backStatus == 2 && girdsNum == 9)
    {
      leftTargetSpeed = 0;
      rightTargetSpeed = 0; // 停车
      MotorControl(0, 0);
      // HAL_Delay(200);
      leftTargetSpeed = 0.10;
      rightTargetSpeed = -0.10; // 原地转右直角弯
      HAL_Delay(1400);
      leftTargetSpeed = 0.10;
      rightTargetSpeed = 0.10; // 恢复直线行驶
      HAL_Delay(2500);
    }
  }
  if (direction == 2 && backStatus == 3 && girdsNum == 0) // 停止工作
  {
    backStatus = 4;
    leftTargetSpeed = 0;
    rightTargetSpeed = 0;
    HAL_Delay(400);
    LED_GREEN_OFF;
    MotorControl(0, 0);
  }
  if (backStatus == 4)
  {
    findTwoLargestIndex(Triangle, &trianglePosition[0], &trianglePosition[1]);
    findTwoLargestIndex(Square, &squarePosition[0], &squarePosition[1]);
    findTwoLargestIndex(Circle, &circlePosition[0], &circlePosition[1]);
    backStatus = 5;
  }
}

void Expand_2()
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
      else if (direction == 2 && backStatus == 3) // 基础1返程
        girdsNum--;
      else if (direction == 0 && mode[1] == 1) // 基础2去程
        girdsNum++;
      else if (direction == 4 && mode[1] == 1) // 基础2返程
        girdsNum--;
    }
  }
  else if (TCRT == 1) // 未扫描到黑线
    girdsNumStatus = 0;
}

void OLEDShow()
{
  sprintf(voltage, "voltage:%.1fV", batteryVoltage);
  OLED_ShowString(0, 0, (char *)voltage, 12, 0);

  sprintf(speedString, "A:%.2fm/s B:%.2fm/s", leftSpeed, rightSpeed);
  OLED_ShowString(0, 1, (char *)speedString, 12, 0);

  sprintf(colorPostion, "x:%d y:%d gird:%d    ", RedX, RedY, girdsNum);
  OLED_ShowString(0, 2, (char *)colorPostion, 12, 0);
  // sprintf(colorPostion, "y:%d ", RedY);
  // OLED_ShowString(60, 1, (char *)colorPostion, 12, 0);

  // sprintf(colorPostion, "girdNum:%d    ", girdsNum);
  // OLED_ShowString(0, 2, (char *)colorPostion, 12, 0);

  sprintf(colorPostion, "triangle: %d, %d        ", trianglePosition[0], trianglePosition[1]);
  OLED_ShowString(0, 3, (char *)colorPostion, 12, 0);
  sprintf(colorPostion, "square: %d, %d        ", squarePosition[0], squarePosition[1]);
  OLED_ShowString(0, 4, (char *)colorPostion, 12, 0);
  sprintf(colorPostion, "circle: %d, %d        ", circlePosition[0], circlePosition[1]);
  OLED_ShowString(0, 5, (char *)colorPostion, 12, 0);

  sprintf(colorPostion, "distance: %d    ", distance);
  OLED_ShowString(0, 6, (char *)colorPostion, 12, 0);
}

void MPU6050_GetData() // 获取MPU6050的数值
{
  while (mpu_dmp_get_data(&pitch, &roll, &yaw))
    ;                                                  // 必须要用while等待，才能读取成功
  MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);           // 得到陀螺仪数据
  printf("data:%.1f,%.1f,%.1f\r\n", roll, pitch, yaw); // 串口1输出采集信息
}

void GetKeyStatus()
{
  if (KEY1 == 0)
  {
    mode[0] = 1;
    mode[1] = 0;
    mode[2] = 0;
    mode[3] = 0;
  }
  else if (KEY2 == 0)
  {
    mode[0] = 0;
    mode[1] = 1;
    mode[2] = 0;
    mode[3] = 0;
  }
  else if (KEY3 == 0)
  {
    mode[0] = 0;
    mode[1] = 0;
    mode[2] = 1;
    mode[3] = 0;
  }
  else if (KEY4 == 0)
  {
    mode[0] = 0;
    mode[1] = 0;
    mode[2] = 0;
    mode[3] = 1;
  }
}

void findTwoLargestIndex(int a[], int *firstIndex, int *secondIndex)
{
  int size = 26;
  *firstIndex = *secondIndex = 0; // 将初始索引设为0
  int largest = 0;

  for (int i = 1; i < size; i++)
  {
    if (a[i] > a[*firstIndex])
    {
      *secondIndex = *firstIndex;
      *firstIndex = i;
      largest = i;
    }
    else if (*secondIndex == *firstIndex || a[i] > a[*secondIndex])
    {
      *secondIndex = i;
    }
  }
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
