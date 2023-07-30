#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "Servo.h"

int angle[2] = {90, 135};
int deltaAngle[2] = {0, 0};
Laser RedLaser;
Laser GreenLaser;

void setServoAngle(int angle_A, int angle_B)
{
    float pulse_width_A = 50.0 + angle_A * 200.0 / 180.0; // 舵机A：上层舵机
    __HAL_TIM_SetCompare(&htim2, Servo_180_Channel, (uint16_t)pulse_width_A);
    float pulse_width_B = 50.0 + angle_B * 200.0 / 270.0; // 舵机B：下层舵机
    __HAL_TIM_SetCompare(&htim2, Servo_270_Channel, (uint16_t)pulse_width_B);
}

void Laser_Init()
{
    RedLaser.x = 0;
    RedLaser.y = 0;
    RedLaser.IS_DETECTED = 0;

    GreenLaser.x = 0;
    GreenLaser.y = 0;
    GreenLaser.IS_DETECTED = 0;
}

void Servo_A_PID_Init(Laser_PID *p)
{
    p->Kp = 0.07;
    p->Ki = 0.01;
    p->Kd = 0.0;
    p->Ur = 90;
    p->PID_is_Enable = 1;
    p->Un = 0;
    p->En_1 = 0;
    p->Integ = 0;
    p->angle = 0;
}

void Servo_B_PID_Init(Laser_PID *p)
{
    p->Kp = 0.07;
    p->Ki = 0.01;
    p->Kd = 0.0;
    p->Ur = 135;
    p->PID_is_Enable = 1;
    p->Un = 0;
    p->En_1 = 0;
    p->Integ = 0;
    p->angle = 0;
}

int Trace_PID_Servo_A(int target_y, int current_y, Laser_PID *p) // 跟踪激光PID
{
    if (p->PID_is_Enable == 1)
    {
        float error = target_y - current_y; // 位置误差

        // 累积误差项
        p->Integ += error;

        // 计算PID输出
        p->angle = p->Kp * error + p->Ki * p->Integ + p->Kd * (error - p->En_1);

        p->En_1 = error; // 保存当前位置误差，供下一次计算使用

        // 输出限幅
        if (p->angle > p->Ur)
        {
            p->angle = p->Ur;
        }
        if (p->angle < -p->Ur)
        {
            p->angle = -p->Ur;
        }
    }
    deltaAngle[0] = (int)p->angle;
    return (int)p->angle;
}

int Trace_PID_Servo_B(int target_x, int current_x, Laser_PID *p) // 跟踪激光PID
{
    if (p->PID_is_Enable == 1)
    {
        float error = target_x - current_x; // 位置误差

        // 累积误差项
        p->Integ += error;

        // 计算PID输出
        p->angle = p->Kp * error + p->Ki * p->Integ + p->Kd * (error - p->En_1);

        p->En_1 = error; // 保存当前位置误差，供下一次计算使用

        // 输出限幅
        if (p->angle > p->Ur)
        {
            p->angle = p->Ur;
        }
        if (p->angle < -p->Ur)
        {
            p->angle = -p->Ur;
        }
    }
    deltaAngle[1] = (int)p->angle;
    return (int)p->angle;
}
