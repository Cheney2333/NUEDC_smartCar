#include "motor.h"
#include "tim.h"

void AMotor_Go() // LeftPostive:LIN1=1,LIN2=0,PB0=1,PB1=0
{
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}
void AMotor_Back() // LeftNegative,LIN1=0,LIN2=1,PB0=1,PB1=1
{
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
}
void AMotor_Stop() // LeftStop,LIN1=LIN2
{
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
}
//-------------------------------------------------
void BMotor_Go() // RightPositive,RIN1=0,RIN2=1,PA6=0,PA7=1
{
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}
void BMotor_Back() // RightNegative,RIN1=1,RIN2=0,PA6=1,PA7=0
{
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
}
void BMotor_Stop() // RightStop,RIN1=RIN2
{
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
}

/**
 *    @brief Control car's movement status
 *    @param motorDirection,LeftMortorPWM,rightMotorPWm
 *    @retval None
 */
void MotorControl(char motorDirection, int AMotorPWM, int BMotorPWM)
{
  switch (motorDirection)
  {
  case 0: // forward
    AMotor_Go();
    BMotor_Go();
    __HAL_TIM_SET_COMPARE(motor_TIM, AMotorChannel, AMotorPWM);
    __HAL_TIM_SET_COMPARE(motor_TIM, BMotorChannel, BMotorPWM);

    break;
  case 1: // backward
    AMotor_Back();
    BMotor_Back();
    __HAL_TIM_SET_COMPARE(motor_TIM, AMotorChannel, AMotorPWM);
    __HAL_TIM_SET_COMPARE(motor_TIM, BMotorChannel, BMotorPWM);
    break;
  case 2: // stop
    AMotor_Stop();
    BMotor_Stop();
    __HAL_TIM_SET_COMPARE(motor_TIM, AMotorChannel, AMotorPWM);
    __HAL_TIM_SET_COMPARE(motor_TIM, BMotorChannel, BMotorPWM);
    break;
  case 3: // left
    AMotor_Back();
    BMotor_Go();
    __HAL_TIM_SET_COMPARE(motor_TIM, AMotorChannel, AMotorPWM);
    __HAL_TIM_SET_COMPARE(motor_TIM, BMotorChannel, BMotorPWM);
  case 4: // right
    AMotor_Go();
    BMotor_Back();
    __HAL_TIM_SET_COMPARE(motor_TIM, AMotorChannel, AMotorPWM);
    __HAL_TIM_SET_COMPARE(motor_TIM, BMotorChannel, BMotorPWM);
  default:
    break;
  }
}

/**
 * @brief  calculate speed based on the obtained encoder pulse value, unit m/s
 * @param pulse
 * @retval speed
 */
float CalActualSpeed(int pulse)
{
  return (float)(0.0101195 * pulse);
}
