#include "pid.h"

extern float targetSpeed;

/**
 * @brief  PID相关参数的初始化
 * @param  PID的结构体指针
 */
void PID_Init(PID *p)
{
	p->Velcity_Kp = 0.0;
	p->Velcity_Ki = 0.1;
	p->Velcity_Kd = 0.0;
	p->Ur = 50;
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

		p->Un += p->Velcity_Kp * (En - p->En_1) + p->Velcity_Ki * En + p->Velcity_Kd * (En - 2 * p->En_1 + p->En_2); // 增量式PID

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
