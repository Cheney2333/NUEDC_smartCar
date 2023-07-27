/**
 * @file    I2C_Simulate.h
 * @brief   模拟IIC驱动头文件
 * @details 该文件包含模拟IIC驱动的所有定义和函数声明
 * @version V1.0
 * @date    2023/7/14
 * @note    None
 */

#ifndef _I2C_Simulate_H
#define _I2C_Simulate_H

/**************************************************************************************************/
//include
#include "stm32f1xx_hal.h"
#include <inttypes.h>

/**************************************************************************************************/
//define
#define IIC_WR	0		//写控制bit
#define IIC_RD	1		//读控制bit

/**************************************************************************************************/
//function
void    IIC_Start(void);
void    IIC_Stop(void);
void    IIC_Send_Byte(uint8_t _ucByte);
uint8_t IIC_Read_Byte(uint8_t ack);
uint8_t IIC_Wait_Ack(void);
void    IIC_Ack(void);
void    IIC_NAck(void);
uint8_t IIC_CheckDevice(uint8_t _Address);
void    IIC_GPIO_Init(void);

#endif
