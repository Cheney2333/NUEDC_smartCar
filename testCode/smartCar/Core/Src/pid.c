#include "pid.h"

/**
 * @brief  PID相关参数的初始化
 * @param  PID的结构体指针
 */
void Speed_PID_Init(PID *p) // 速度环
{
	p->Kp = 25.0;
	p->Ki = 2.5;
	p->Kd = 2.5;
	p->Ur = 100;
	p->PID_is_Enable = 1;
	p->Un = 3;
	p->En_1 = 0;
	p->En_2 = 0;
	p->PWM = 0;
}
void Position_PID_Init(PID_POSITION *p) // 位置环
{
	p->Kp = 0.8;
	p->Ki = 0.0;
	p->Kd = 0.5;
	p->Ur = 0.6; // 最大速度0.40m/s
	p->PID_is_Enable = 1;
	p->Un = 0;
	p->En_1 = 0;
	p->Integ = 0;
	p->targetSpeed = 0;
}
void Trail_PID_Init(PID *p) // 循迹
{
	p->Kp = 0.32;
	p->Ki = 0.017;
	p->Kd = 0.0;
	p->Ur = 0.075;
	p->PID_is_Enable = 1;
	p->Un = 0;
	p->En_1 = 0;
	p->En_2 = 0;
	p->PWM = 0;
}
/**
 * @brief  PID相关参数的初始化
 * @param  targetSpeed目标速度值，PID的结构体指针p
 */
void Speed_PID(float targetSpeed, float currentSpeed, PID *p)
{
	if (p->PID_is_Enable == 1)
	{
		float En = targetSpeed - currentSpeed; // 误差值

		p->Un += p->Kp * (En - p->En_1) + p->Ki * En + p->Kd * (En - 2 * p->En_1 + p->En_2); // 增量式PID

		p->En_2 = p->En_1;
		p->En_1 = En;

		p->PWM = (int)p->Un;

		/*输出限幅*/
		if (p->PWM > p->Ur)
			p->PWM = p->Ur;
		if (p->PWM < -p->Ur)
			p->PWM = -p->Ur;
	}
	else
	{
		Speed_PID_Init(p);
	}
}
float Position_PID(float targetPosition, float currentPosition, PID_POSITION *p) // 此处的位置即为轮子圈数
{
	if (p->PID_is_Enable == 1)
	{
		float error = targetPosition - currentPosition; // 位置误差

		// 累积误差项
		p->Integ += error;

		// 计算PID输出
		p->targetSpeed = p->Kp * error + p->Ki * p->Integ + p->Kd * (error - p->En_1);

		p->En_1 = error; // 保存当前位置误差，供下一次计算使用

		// 输出限幅
		if (p->targetSpeed > p->Ur)
		{
			p->targetSpeed = p->Ur;
		}
		if (p->targetSpeed < -p->Ur)
		{
			p->targetSpeed = -p->Ur;
		}
	}
	else
	{
		Position_PID_Init(p);
	}

	return p->targetSpeed;
}
void Trail_PID(int currentX, PID *p)
{
	int centerX = 160;
	if (p->PID_is_Enable == 1)
	{
		int En = currentX - centerX; // 误差值

		p->Un += p->Kp * (En - p->En_1) + p->Ki * En + p->Kd * (En - 2 * p->En_1 + p->En_2); // 增量式PID

		p->En_2 = p->En_1;
		p->En_1 = En;

		p->Un = (int)p->Un; // 目标速度变化量

		/*输出限幅*/
		if (p->Un > p->Ur)
			p->Un = p->Ur;
		if (p->Un < -p->Ur)
			p->Un = -p->Ur;
	}
	else
	{
		Trail_PID_Init(p);
	}
}
