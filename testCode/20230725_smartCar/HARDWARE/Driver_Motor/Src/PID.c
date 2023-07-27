/**
 * @file PID.c
 * @brief PID控制源文件
 * @version 1.0
 * @date 2023/7/17
 * @note None
*/

/***************************************************************************/
//include
#include "PID.h"

/***************************************************************************/
//function
/**
 * @brief  速度PID相关参数的初始化
 * @param  p[PID*] PID的结构体指针
 * @return None
 */
void Speed_PID_Init(PID *p)
{
	p->Kp = 25.0;
	p->Ki = 2.5;
	p->Kd = 2.5;
	p->Ur = 60;
	p->PID_is_Enable = 1;
	p->Un = 3;
	p->En_1 = 0;
	p->En_2 = 0;
	p->PWM = 0;
}

/**
 * @brief  速度PID控制
 * @param  targetSpeed[float] 目标速度
 * @param  currentSpeed[float] 当前速度
 * @param  p[PID*] PID的结构体指针
 * @return None
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

/**
 * @brief  循迹PID相关参数的初始化
 * @param  p[PID*] PID的结构体指针
 * @return None
 */
void Trail_PID_Init(PID *p)
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
 * @brief  循迹PID控制
 * @param  currentX[int] 当前X坐标
 * @param  p[PID*] PID的结构体指针
 * @return None
 */
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
