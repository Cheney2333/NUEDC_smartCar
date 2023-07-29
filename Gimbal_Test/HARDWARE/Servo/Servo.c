#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "Servo.h"

void setServoAngle(int angle_A, int angle_B)
{
    float pulse_width_A = 50.0 + angle_A * 200.0 / 180.0; // 舵机A：上层舵机
    __HAL_TIM_SetCompare(&htim2, Servo_180_Channel, (uint16_t)pulse_width_A);
    float pulse_width_B = 50.0 + angle_B * 200.0 / 270.0; // 舵机B：下层舵机
    __HAL_TIM_SetCompare(&htim2, Servo_270_Channel, (uint16_t)pulse_width_B);
}
