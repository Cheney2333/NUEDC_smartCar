/**
 * @file    MPU_6050.c
 * @brief   MPU6050驱动代码(适用于STM32F103x)
 * @details 包含MPU6050的初始化、读写、IIC驱动等函数
 * @version V1.1
 * @date    2023/7/19
 * @note    None
 */

/**************************************************************************************************/
//include
#include "MPU.h"

/**************************************************************************************************/
//variable
struct MPU mpu;
uint8_t mpu_struct_init_flag = 0;

/**************************************************************************************************/
//function
/** 
 * @brief   MPU6050结构体初始化
 * @param   None
 * @return  None
 * @note    None
 */
void MPU6050_Struct_Init(void)
{
    if(mpu_struct_init_flag == 0)
    {
        mpu.mpu6050.pitch               = 0;
        mpu.mpu6050.roll                = 0;
        mpu.mpu6050.yaw                 = 0;
        mpu.mpu6050.gyrox               = 0;
        mpu.mpu6050.gyroy               = 0;
        mpu.mpu6050.gyroz               = 0;

        mpu.mpu6050.Write_Len           = MPU6050_Write_Len;
        mpu.mpu6050.Read_Len            = MPU6050_Read_Len;
        mpu.mpu6050.Write_Byte          = MPU6050_Write_Byte;
        mpu.mpu6050.Read_Byte           = MPU6050_Read_Byte;
        mpu.mpu6050.Set_GyroFsr         = MPU6050_Set_GyroFsr;
        mpu.mpu6050.Set_AccelFsr        = MPU6050_Set_AccelFsr;
        mpu.mpu6050.Set_LPF             = MPU6050_Set_LPF;
        mpu.mpu6050.Set_Rate            = MPU6050_Set_Rate;
        //mpu.mpu6050.Set_Fifo            = MPU6050_Set_Fifo;   //MPU_6050.c中未定义
        mpu.mpu6050.Get_Temperature     = MPU6050_Get_Temperature;
        mpu.mpu6050.Get_Gyroscope       = MPU6050_Get_Gyroscope;
        mpu.mpu6050.Get_Accelerometer   = MPU6050_Get_Accelerometer;

        mpu_struct_init_flag            = 1;
    }
}

/** 
 * @brief   IIC初始化
 * @param   None
 * @return  0:成功 1:失败
 * @note    None
 */
uint8_t MPU6050_Init(void)
{
    if(mpu_struct_init_flag == 0)
        MPU6050_Struct_Init();

    uint8_t res;

    MPU6050_IIC_Init();                              // 初始化IIC总线
    MPU6050_Write_Byte(MPU6050_PWR_MGMT1_REG, 0x80); // 复位MPU6050
    delay_ms(100);
    MPU6050_Write_Byte(MPU6050_PWR_MGMT1_REG, 0x00); // 唤醒MPU6050
    MPU6050_Set_GyroFsr(MPU6050_GyroFsr2000dps);     // 陀螺仪传感器,±2000dps
    MPU6050_Set_AccelFsr(MPU6050_AccelFsr2g);        // 加速度传感器,±2g
    MPU6050_Set_Rate(50);                            // 设置采样率50Hz
    MPU6050_Write_Byte(MPU6050_INT_EN_REG, 0x00);    // 关闭所有中断
    MPU6050_Write_Byte(MPU6050_USER_CTRL_REG, 0x00); // I2C主模式关闭
    MPU6050_Write_Byte(MPU6050_FIFO_EN_REG, 0x00);   // 关闭FIFO
    MPU6050_Write_Byte(MPU6050_INTBP_CFG_REG, 0x80); // INT引脚低电平有效
    res = MPU6050_Read_Byte(MPU6050_DEVICE_ID_REG);
    if (res == MPU6050_ADDR) // 器件ID正确
    {
        MPU6050_Write_Byte(MPU6050_PWR_MGMT1_REG, 0x01); // 设置CLKSEL,PLL X轴为参考
        MPU6050_Write_Byte(MPU6050_PWR_MGMT2_REG, 0x00); // 加速度与陀螺仪都工作
        MPU6050_Set_Rate(50);                            // 设置采样率为50Hz
    }
    else
        return 1;
    return 0;
}

/**
 * @brief   设置MPU6050陀螺仪传感器满量程范围
 * @param   fsr MPU_Fsr250dps,±250dps; MPU_Fsr500dps,±500dps; MPU_Fsr1000dps,±1000dps; MPU_Fsr2000dps,±2000dps
 * @return  0:成功 其他:失败
 * @note    None
 */
uint8_t MPU6050_Set_GyroFsr(uint8_t fsr)
{
    return MPU6050_Write_Byte(MPU6050_GYRO_CFG_REG, fsr << 3); // 设置陀螺仪满量程范围
}

/**
 * @brief   设置MPU6050加速度传感器满量程范围
 * @param   fsr MPU_AccelFsr2g,±2g; MPU_AccelFsr4g,±4g; MPU_AccelFsr8g,±8g; MPU_AccelFsr16g,±16g
 * @return  0:成功 其他:失败
 * @note    None
 */
uint8_t MPU6050_Set_AccelFsr(uint8_t fsr)
{
    return MPU6050_Write_Byte(MPU6050_ACCEL_CFG_REG, fsr << 3); // 设置加速度传感器满量程范围
}

/**
 * @brief   设置MPU6050的数字低通滤波器
 * @param   lpf 数字低通滤波频率(Hz)
 * @return  0:成功 其他:失败
 * @note    None
 */
uint8_t MPU6050_Set_LPF(uint16_t lpf)
{
    uint8_t data = 0;
    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return MPU6050_Write_Byte(MPU6050_CFG_REG, data); // 设置数字低通滤波器
}

/**
 * @brief   设置MPU6050的采样率
 * @param   rate 4~1000(Hz)
 * @return  0:成功 其他:失败
 * @note    None
 */
uint8_t MPU6050_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;
    data = 1000 / rate - 1;
    data = MPU6050_Write_Byte(MPU6050_SAMPLE_RATE_REG, data); // 设置数字低通滤波器
    return MPU6050_Set_LPF(rate / 2);                     // 自动设置LPF为采样率的一半
}

/**
 * @brief   得到温度值
 * @param   None
 * @return  温度值(扩大了100倍)
 * @note    None
 */
short MPU6050_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
    float temp;
    MPU6050_Read_Len(MPU6050_ADDR, MPU6050_TEMP_OUTH_REG, 2, buf);
    raw = ((uint16_t)buf[0] << 8) | buf[1];
    temp = 36.53 + ((double)raw) / 340;
    return temp * 100;
}

/**
 * @brief   得到陀螺仪值(原始值)
 * @param   gx,gy,gz 陀螺仪x,y,z轴的原始读数(带符号)
 * @return  0:成功 其他:失败
 * @note    None
 */
uint8_t MPU6050_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    uint8_t buf[6], res;
    res = MPU6050_Read_Len(MPU6050_ADDR, MPU6050_GYRO_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *gx = ((uint16_t)buf[0] << 8) | buf[1];
        *gy = ((uint16_t)buf[2] << 8) | buf[3];
        *gz = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
}

/**
 * @brief   得到加速度值(原始值)
 * @param   ax,ay,az 加速度x,y,z轴的原始读数(带符号)
 * @return  0:成功 其他:失败
 * @note    None
 */
uint8_t MPU6050_Get_Accelerometer(short *ax, short *ay, short *az)
{
    uint8_t buf[6], res;
    res = MPU6050_Read_Len(MPU6050_ADDR, MPU6050_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *ax = ((uint16_t)buf[0] << 8) | buf[1];
        *ay = ((uint16_t)buf[2] << 8) | buf[3];
        *az = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
    ;
}

/**
 * @brief   IIC连续写
 * @param   addr 器件地址
 * @param   reg 寄存器地址
 * @param   len 写入长度
 * @param   buf 数据区
 * @return  0:成功 其他:失败
 * @note    None
 */
uint8_t MPU6050_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint8_t i;
    MPU6050_IIC_Start();
    MPU6050_IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
    if (MPU6050_IIC_Wait_Ack())             // 等待应答
    {
        MPU6050_IIC_Stop();
        return 1;
    }
    MPU6050_IIC_Send_Byte(reg); // 写寄存器地址
    MPU6050_IIC_Wait_Ack();     // 等待应答
    for (i = 0; i < len; i++)
    {
        MPU6050_IIC_Send_Byte(buf[i]); // 发送数据
        if (MPU6050_IIC_Wait_Ack())    // 等待ACK
        {
            MPU6050_IIC_Stop();
            return 1;
        }
    }
    MPU6050_IIC_Stop();
    return 0;
}

/**
 * @brief   IIC连续读
 * @param   addr 器件地址
 * @param   reg 寄存器地址
 * @param   len 读取长度
 * @param   buf 数据区
 * @return  0:成功 其他:失败
 * @note    None
 */
uint8_t MPU6050_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    MPU6050_IIC_Start();
    MPU6050_IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
    if (MPU6050_IIC_Wait_Ack())             // 等待应答
    {
        MPU6050_IIC_Stop();
        return 1;
    }
    MPU6050_IIC_Send_Byte(reg); // 写寄存器地址
    MPU6050_IIC_Wait_Ack();     // 等待应答
    MPU6050_IIC_Start();
    MPU6050_IIC_Send_Byte((addr << 1) | 1); // 发送器件地址+读命令
    MPU6050_IIC_Wait_Ack();                 // 等待应答
    while (len)
    {
        if (len == 1)
            *buf = MPU6050_IIC_Read_Byte(0); // 读数据,发送nACK
        else
            *buf = MPU6050_IIC_Read_Byte(1); // 读数据,发送ACK
        len--;
        buf++;
    }
    MPU6050_IIC_Stop(); // 产生一个停止条件
    return 0;
}

/**
 * @brief   IIC写一个字节
 * @param   reg 寄存器地址
 * @param   data 数据
 * @return  0:成功 其他:失败
 * @note    None
 */
uint8_t MPU6050_Write_Byte(uint8_t reg, uint8_t data)
{
    MPU6050_IIC_Start();
    MPU6050_IIC_Send_Byte((MPU6050_ADDR << 1) | 0); // 发送器件地址+写命令
    if (MPU6050_IIC_Wait_Ack())                 // 等待应答
    {
        MPU6050_IIC_Stop();
        return 1;
    }
    MPU6050_IIC_Send_Byte(reg);  // 写寄存器地址
    MPU6050_IIC_Wait_Ack();      // 等待应答
    MPU6050_IIC_Send_Byte(data); // 发送数据
    if (MPU6050_IIC_Wait_Ack())  // 等待ACK
    {
        MPU6050_IIC_Stop();
        return 1;
    }
    MPU6050_IIC_Stop();
    return 0;
}

/**
 * @brief   IIC读一个字节
 * @param   reg 寄存器地址
 * @return  读到的数据
 * @note    None
 */
uint8_t MPU6050_Read_Byte(uint8_t reg)
{
    uint8_t res;
    MPU6050_IIC_Start();
    MPU6050_IIC_Send_Byte((MPU6050_ADDR << 1) | 0); // 发送器件地址+写命令
    MPU6050_IIC_Wait_Ack();                     // 等待应答
    MPU6050_IIC_Send_Byte(reg);                 // 写寄存器地址
    MPU6050_IIC_Wait_Ack();                     // 等待应答
    MPU6050_IIC_Start();
    MPU6050_IIC_Send_Byte((MPU6050_ADDR << 1) | 1); // 发送器件地址+读命令
    MPU6050_IIC_Wait_Ack();                     // 等待应答
    res = MPU6050_IIC_Read_Byte(0);             // 读取数据,发送nACK
    MPU6050_IIC_Stop();                         // 产生一个停止条件
    return res;
}
