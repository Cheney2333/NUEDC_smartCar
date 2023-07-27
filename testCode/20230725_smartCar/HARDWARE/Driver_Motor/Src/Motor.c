/**
 * @file Motor.c
 * @brief 适用于MG513编码点击的电机驱动源文件
 * @version 1.1
 * @date 2023/7/19
 * @note None
*/

/***************************************************************************/
//include
#include "Motor.h"

/***************************************************************************/
//function

/**
 * @brief 电机A正转
 */
void AMotor_Go() // LeftPostive:LIN1=1,LIN2=0,PB0=1,PB1=0
{
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 电机A反转
 */
void AMotor_Back() // LeftNegative,LIN1=0,LIN2=1,PB0=1,PB1=1
{
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
}

/**
 * @brief 电机A停止
 */
void AMotor_Stop() // LeftStop,LIN1=LIN2
{
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
}

/**
 * @brief 电机B正转
 */
void BMotor_Go() // RightPositive,RIN1=0,RIN2=1,PA6=0,PA7=1
{
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 电机B反转
 */
void BMotor_Back() // RightNegative,RIN1=1,RIN2=0,PA6=1,PA7=0
{
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
}

/**
 * @brief 电机B停止
 */
void BMotor_Stop() // RightStop,RIN1=RIN2
{
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
}

/**
 * @brief  读取定时器2和定时器3的计数值(编码器脉冲值)
 * @param  None
 * @return None
 */
void GetEncoderPulse()
{
  encoderPulse[0] = ((short)__HAL_TIM_GET_COUNTER(&htim2)); // 电机A,左轮
  encoderPulse[1] = ((short)__HAL_TIM_GET_COUNTER(&htim3));

  __HAL_TIM_GET_COUNTER(&htim2) = 0; // 计数值重新清零
  __HAL_TIM_GET_COUNTER(&htim3) = 0;
}

/**
 * @brief  电机控制
 * @param  AMotorPWM[int] 电机A的PWM值
 * @param  BMotorPWM[int] 电机B的PWM值
 * @return None
 */
void MotorControl(int AMotorPWM, int BMotorPWM)
{
  if (AMotorPWM > 0 && BMotorPWM > 0) // 前进
  {
    AMotor_Go();
    BMotor_Go();
    __HAL_TIM_SET_COMPARE(motor_TIM, AMotorChannel, AMotorPWM);
    __HAL_TIM_SET_COMPARE(motor_TIM, BMotorChannel, BMotorPWM);
  }
  if (AMotorPWM == 0 && BMotorPWM == 0) // 停车
  {
    AMotor_Stop();
    BMotor_Stop();
    __HAL_TIM_SET_COMPARE(motor_TIM, AMotorChannel, AMotorPWM);
    __HAL_TIM_SET_COMPARE(motor_TIM, BMotorChannel, BMotorPWM);
  }
  if (AMotorPWM < 0 && BMotorPWM < 0) // 后退
  {
    AMotor_Back();
    BMotor_Back();
    __HAL_TIM_SET_COMPARE(motor_TIM, AMotorChannel, -AMotorPWM);
    __HAL_TIM_SET_COMPARE(motor_TIM, BMotorChannel, -BMotorPWM);
  }
  if (AMotorPWM > 0 && BMotorPWM < 0) // 原地右转
  {
    AMotor_Go();
    BMotor_Back();
    __HAL_TIM_SET_COMPARE(motor_TIM, AMotorChannel, AMotorPWM);
    __HAL_TIM_SET_COMPARE(motor_TIM, BMotorChannel, -BMotorPWM);
  }
  if (AMotorPWM < 0 && BMotorPWM > 0) // 原地左转
  {
    AMotor_Back();
    BMotor_Go();
    __HAL_TIM_SET_COMPARE(motor_TIM, AMotorChannel, -AMotorPWM);
    __HAL_TIM_SET_COMPARE(motor_TIM, BMotorChannel, BMotorPWM);
  }
}

/**
 * @brief   计算实际速度
 * @param   pulse[int] 编码器脉冲值
 * @return  实际速度
*/
float CalActualSpeed(int pulse)
{
  return (float)(0.01349 * pulse); // unit: m/s
}

/**
 * @brief  左直角弯
 * @param  refrenceAngle[float] 参考角度
 * @return None
*/
void LeftBend(float refrenceAngle)
{
  while ((mpu.mpu6050.pitch) - refrenceAngle < 89.5)
  {
    leftTargetSpeed = LeftBackSpeed;
    rightTargetSpeed = RightForwardSpeed;
    MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
  }
  MotorControl(0, 0);
}

/**
 * @brief  右直角弯
 * @param  refrenceAngle[float] 参考角度
 * @return None
*/
void RightBend(float refrenceAngle)
{
  while ((mpu.mpu6050.pitch) - refrenceAngle > -89.5)
  {
    leftTargetSpeed = LeftForwardSpeed;
    rightTargetSpeed = RightBackSpeed;
    MotorControl(leftMotor_PID.PWM, rightMotor_PID.PWM);
  }
  MotorControl(0, 0);
}
