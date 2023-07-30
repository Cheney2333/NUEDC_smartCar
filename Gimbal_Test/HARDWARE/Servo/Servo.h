/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SERVO_H__
#define __SERVO_H__

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        int x;
        int y;
        int IS_DETECTED;
    } Laser;

    typedef struct
    {
        float Kp;
        float Ki;
        float Kd;
        float Ur; // 限幅值

        int PID_is_Enable; // PID使能
        float Un;          // 期望输出值
        float En_1;        // 上一次的误差值
        float Integ;       // 累计误差，只在位置环用到
        float angle;       // 输出目标速度
    } Laser_PID;

    void setServoAngle(int angle_A, int angle_B);
    void Laser_Init(void);
    void Servo_A_PID_Init(Laser_PID *p);
    void Servo_B_PID_Init(Laser_PID *p);                              // PID值初始化
    int Trace_PID_Servo_A(int target_y, int current_y, Laser_PID *p); // 跟踪激光PID
    int Trace_PID_Servo_B(int target_x, int current_x, Laser_PID *p); // 跟踪激光PID

#define K210_SCREEN_CENTER_X 160
#define K210_SCREEN_CENTER_Y 120

#ifdef __cplusplus
}
#endif
#endif /*__SERVO_H__ */
