#include "pid.h"

/**
 * @brief  PID相关参数的初始化
 * @param  PID的结构体指针
 */
void PID_Init(PID *p)
{
	p->Kp = 2.5;
	p->Ki = 0.5;
	p->Kd = 0.0;
	p->Ur = 60;
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

		p->PWM = p->Un;

		/*输出限幅*/
		if (p->PWM > p->Ur)
			p->PWM = p->Ur;
		if (p->PWM < -p->Ur)
			p->PWM = -p->Ur;
	}
	else
	{
		PID_Init(p);
	}
}
