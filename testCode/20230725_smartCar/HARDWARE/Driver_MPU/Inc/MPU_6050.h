/*
 * @file    MPU_6050.h
 * @brief   MPU6050 头文件
 * @details 包含MPU6050的所有函数声明和宏定义
 * @version V1.1
 * @date    2023/7/19
 * @note    None
 */
#ifndef __MPU6050_6050_H
#define __MPU6050_6050_H

/*************************************************************************************************/
//include
//#include "stm32f1xx_hal.h"
#include "I2C_Simulate.h"

/*************************************************************************************************/
//define
#define delay_ms                    HAL_Delay
#define MPU6050_IIC_Init            IIC_GPIO_Init
#define MPU6050_IIC_Start           IIC_Start
#define MPU6050_IIC_Stop            IIC_Stop
#define MPU6050_IIC_Send_Byte       IIC_Send_Byte
#define MPU6050_IIC_Read_Byte       IIC_Read_Byte
#define MPU6050_IIC_Wait_Ack        IIC_Wait_Ack

#define MPU6050_ACCEL_OFFS_REG		0x06	//加速度计偏移量寄存器
#define MPU6050_PROD_ID_REG			0x0C	//产品ID寄存器
#define MPU6050_SELF_TESTX_REG      0x0D    //自检寄存器X
#define MPU6050_SELF_TESTY_REG      0x0E    //自检寄存器Y
#define MPU6050_SELF_TESTZ_REG      0x0F    //自检寄存器Z
#define MPU6050_SELF_TESTA_REG      0x10    //自检寄存器A
#define MPU6050_SAMPLE_RATE_REG     0x19    //采样频率分频器
#define MPU6050_CFG_REG             0x1A    //配置寄存器
#define MPU6050_GYRO_CFG_REG        0x1B    //陀螺仪配置寄存器
#define MPU6050_ACCEL_CFG_REG       0x1C    //加速度计配置寄存器
#define MPU6050_MOTION_DET_REG      0x1F    //运动检测阀值设置寄存器
#define MPU6050_FIFO_EN_REG         0x23    //FIFO使能寄存器
#define MPU6050_I2CMST_CTRL_REG     0x24    //IIC主机控制寄存器
#define MPU6050_I2CSLV0_ADDR_REG    0x25    //IIC从机0器件地址寄存器
#define MPU6050_I2CSLV0_REG         0x26    //IIC从机0数据地址寄存器
#define MPU6050_I2CSLV0_CTRL_REG    0x27    //IIC从机0控制寄存器
#define MPU6050_I2CSLV1_ADDR_REG    0x28    //IIC从机1器件地址寄存器
#define MPU6050_I2CSLV1_REG         0x29    //IIC从机1数据地址寄存器
#define MPU6050_I2CSLV1_CTRL_REG    0x2A    //IIC从机1控制寄存器
#define MPU6050_I2CSLV2_ADDR_REG    0x2B    //IIC从机2器件地址寄存器
#define MPU6050_I2CSLV2_REG         0x2C    //IIC从机2数据地址寄存器
#define MPU6050_I2CSLV2_CTRL_REG    0x2D    //IIC从机2控制寄存器
#define MPU6050_I2CSLV3_ADDR_REG    0x2E    //IIC从机3器件地址寄存器
#define MPU6050_I2CSLV3_REG         0x2F    //IIC从机3数据地址寄存器
#define MPU6050_I2CSLV3_CTRL_REG    0x30    //IIC从机3控制寄存器
#define MPU6050_I2CSLV4_ADDR_REG    0x31    //IIC从机4器件地址寄存器
#define MPU6050_I2CSLV4_REG         0x32    //IIC从机4数据地址寄存器
#define MPU6050_I2CSLV4_DO_REG      0x33    //IIC从机4写数据寄存器
#define MPU6050_I2CSLV4_CTRL_REG    0x34    //IIC从机4控制寄存器
#define MPU6050_I2CSLV4_DI_REG      0x35    //IIC从机4读数据寄存器

#define MPU6050_I2CMST_STA_REG      0x36    //IIC主机状态寄存器
#define MPU6050_INTBP_CFG_REG       0x37    //中断/旁路设置寄存器
#define MPU6050_INT_EN_REG          0x38    //中断使能寄存器
#define MPU6050_INT_STA_REG         0x3A    //中断状态寄存器

#define MPU6050_ACCEL_XOUTH_REG     0x3B    //加速度值,X轴高8位寄存器
#define MPU6050_ACCEL_XOUTL_REG     0x3C    //加速度值,X轴低8位寄存器
#define MPU6050_ACCEL_YOUTH_REG     0x3D    //加速度值,Y轴高8位寄存器
#define MPU6050_ACCEL_YOUTL_REG     0x3E    //加速度值,Y轴低8位寄存器
#define MPU6050_ACCEL_ZOUTH_REG     0x3F    //加速度值,Z轴高8位寄存器
#define MPU6050_ACCEL_ZOUTL_REG     0x40    //加速度值,Z轴低8位寄存器

#define MPU6050_TEMP_OUTH_REG       0x41    //温度值高八位寄存器
#define MPU6050_TEMP_OUTL_REG       0x42    //温度值低8位寄存器

#define MPU6050_GYRO_XOUTH_REG      0x43    //陀螺仪值,X轴高8位寄存器
#define MPU6050_GYRO_XOUTL_REG      0x44    //陀螺仪值,X轴低8位寄存器
#define MPU6050_GYRO_YOUTH_REG      0x45    //陀螺仪值,Y轴高8位寄存器
#define MPU6050_GYRO_YOUTL_REG      0x46    //陀螺仪值,Y轴低8位寄存器
#define MPU6050_GYRO_ZOUTH_REG      0x47    //陀螺仪值,Z轴高8位寄存器
#define MPU6050_GYRO_ZOUTL_REG      0x48    //陀螺仪值,Z轴低8位寄存器

#define MPU6050_I2CSLV0_DO_REG      0x63    //IIC从机0数据寄存器
#define MPU6050_I2CSLV1_DO_REG      0x64    //IIC从机1数据寄存器
#define MPU6050_I2CSLV2_DO_REG      0x65    //IIC从机2数据寄存器
#define MPU6050_I2CSLV3_DO_REG      0x66    //IIC从机3数据寄存器

#define MPU6050_I2CMST_DELAY_REG    0x67    //IIC主机延时管理寄存器
#define MPU6050_SIGPATH_RST_REG     0x68    //信号通道复位寄存器
#define MPU6050_MDETECT_CTRL_REG    0x69    //运动检测控制寄存器 
#define MPU6050_USER_CTRL_REG       0x6A    //用户控制寄存器
#define MPU6050_PWR_MGMT1_REG       0x6B    //电源管理寄存器1
#define MPU6050_PWR_MGMT2_REG       0x6C    //电源管理寄存器2
#define MPU6050_FIFO_CNTH_REG       0x72    //FIFO计数寄存器高八位
#define MPU6050_FIFO_CNTL_REG       0x73    //FIFO计数寄存器低八位
#define MPU6050_FIFO_RW_REG         0x74    //FIFO读写寄存器
#define MPU6050_DEVICE_ID_REG       0x75    //器件ID寄存器

/* MPU6050的ADO引脚会影响其的IIC地址
 * 当ADO为低电平时, IIC地址为0x68
 * 当ADO为高电平时, IIC地址为0x69
 */
#define MPU6050_ADDR                0x68    //MPU6050器件地址

#define MPU6050_READ                0xD1    //定义读写命令
#define MPU6050_WRITE               0xD0    //定义读写命令

#define MPU6050_GyroFsr250dps       0x0
#define MPU6050_GyroFsr500dps       0x1
#define MPU6050_GyroFsr1000dps      0x2
#define MPU6050_GyroFsr2000dps      0x3

#define MPU6050_AccelFsr2g          0x0
#define MPU6050_AccelFsr4g          0x1
#define MPU6050_AccelFsr8g          0x2
#define MPU6050_AccelFsr16g         0x3

/*************************************************************************************************/
//struct
typedef struct MPU_6050
{
    float pitch, roll, yaw;    // 欧拉角
    short gyrox, gyroy, gyroz; // 陀螺仪原始数据

/**
 * @brief   IIC连续写
 * @param   addr 器件地址
 * @param   reg 寄存器地址
 * @param   len 写入长度
 * @param   buf 数据区
 * @return  0:成功 其他:失败
 * @note    None
 */
    uint8_t (*Write_Len)(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf); 
    
/**
 * @brief   IIC连续读
 * @param   addr 器件地址
 * @param   reg 寄存器地址
 * @param   len 读取长度
 * @param   buf 数据区
 * @return  0:成功 其他:失败
 * @note    None
 */
    uint8_t (*Read_Len)(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);  
    
/**
 * @brief   IIC写一个字节
 * @param   reg 寄存器地址
 * @param   data 数据
 * @return  0:成功 其他:失败
 * @note    None
 */
    uint8_t (*Write_Byte)(uint8_t reg, uint8_t data);                           
    
/**
 * @brief   IIC读一个字节
 * @param   reg 寄存器地址
 * @return  读到的数据
 * @note    None
 */
    uint8_t (*Read_Byte)(uint8_t reg);                                          

/**
 * @brief   设置MPU6050陀螺仪传感器满量程范围
 * @param   fsr MPU_Fsr250dps,±250dps; MPU_Fsr500dps,±500dps; MPU_Fsr1000dps,±1000dps; MPU_Fsr2000dps,±2000dps
 * @return  0:成功 其他:失败
 * @note    None
 */
    uint8_t (*Set_GyroFsr)(uint8_t fsr);

/**
 * @brief   设置MPU6050加速度传感器满量程范围
 * @param   fsr MPU_AccelFsr2g,±2g; MPU_AccelFsr4g,±4g; MPU_AccelFsr8g,±8g; MPU_AccelFsr16g,±16g
 * @return  0:成功 其他:失败
 * @note    None
 */
    uint8_t (*Set_AccelFsr)(uint8_t fsr);

/**
 * @brief   设置MPU6050的数字低通滤波器
 * @param   lpf 数字低通滤波频率(Hz)
 * @return  0:成功 其他:失败
 * @note    None
 */
    uint8_t (*Set_LPF)(uint16_t lpf);

/**
 * @brief   设置MPU6050的采样率
 * @param   rate 4~1000(Hz)
 * @return  0:成功 其他:失败
 * @note    None
 */
    uint8_t (*Set_Rate)(uint16_t rate);
    //uint8_t (*Set_Fifo)(uint8_t sens);

/**
 * @brief   得到温度值
 * @param   None
 * @return  温度值(扩大了100倍)
 * @note    None
 */
    short   (*Get_Temperature)(void);

/**
 * @brief   得到陀螺仪值(原始值)
 * @param   gx,gy,gz 陀螺仪x,y,z轴的原始读数(带符号)
 * @return  0:成功 其他:失败
 * @note    None
 */
    uint8_t (*Get_Gyroscope)(short *gx, short *gy, short *gz);

/**
 * @brief   得到加速度值(原始值)
 * @param   ax,ay,az 加速度x,y,z轴的原始读数(带符号)
 * @return  0:成功 其他:失败
 * @note    None
 */
    uint8_t (*Get_Accelerometer)(short *ax, short *ay, short *az);
}MPU_6050_Handler;

/*************************************************************************************************/
//function

void    MPU6050_Struct_Init(void);

uint8_t MPU6050_Init(void);
uint8_t MPU6050_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf); 
uint8_t MPU6050_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);  
uint8_t MPU6050_Write_Byte(uint8_t reg, uint8_t data);                           
uint8_t MPU6050_Read_Byte(uint8_t reg);                                          

uint8_t MPU6050_Set_GyroFsr(uint8_t fsr);
uint8_t MPU6050_Set_AccelFsr(uint8_t fsr);
uint8_t MPU6050_Set_LPF(uint16_t lpf);
uint8_t MPU6050_Set_Rate(uint16_t rate);
//uint8_t MPU6050_Set_Fifo(uint8_t sens);

short   MPU6050_Get_Temperature(void);
uint8_t MPU6050_Get_Gyroscope(short *gx, short *gy, short *gz);
uint8_t MPU6050_Get_Accelerometer(short *ax, short *ay, short *az);

#endif
