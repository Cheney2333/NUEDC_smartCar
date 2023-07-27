/**
 * @file    IIC.c
 * @brief   模拟IIC驱动
 * @details None
 * @version V1.0
 * @date    2023/7/14
 * @note    None
 */

/**************************************************************************************************/
//include
#include "stm32f1xx_hal.h"
#include "I2C_Simulate.h"

/**************************************************************************************************/
//define
#define GPIO_PORT_IIC     GPIOB                       // GPIO端口
#define RCC_IIC_ENABLE    __HAL_RCC_GPIOB_CLK_ENABLE()       // GPIO端口时钟
#define IIC_SCL_PIN       GPIO_PIN_8                  // 连接到SCL时钟线的GPIO
#define IIC_SDA_PIN       GPIO_PIN_9                  // 连接到SDA数据线的GPIO

#if 1	//这个分支选择HAL库函数实现IO读写
    #define IIC_SCL_1()     HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SCL_PIN, GPIO_PIN_SET)		// SCL = 1
    #define IIC_SCL_0()     HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SCL_PIN, GPIO_PIN_RESET)   // SCL = 0

    #define IIC_SDA_1()     HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SDA_PIN, GPIO_PIN_SET)		// SDA = 1
    #define IIC_SDA_0()     HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SDA_PIN, GPIO_PIN_RESET)	// SDA = 0

    #define IIC_SDA_READ()  HAL_GPIO_ReadPin(GPIO_PORT_IIC, IIC_SDA_PIN)	                //读SDA线状态
#else	//这个分支选择直接操作寄存器实现IO读写（标准库）
/*　注意：如下写法，在IAR最高级别优化时，会被编译器错误优化 */
    #define IIC_SCL_1()     GPIO_PORT_IIC->BSRR = IIC_SCL_PIN				                //SCL = 1
    #define IIC_SCL_0()     GPIO_PORT_IIC->BRR = IIC_SCL_PIN				                //SCL = 0

    #define IIC_SDA_1()     GPIO_PORT_IIC->BSRR = IIC_SDA_PIN				                //SDA = 1 
    #define IIC_SDA_0()     GPIO_PORT_IIC->BRR = IIC_SDA_PIN				                //SDA = 0

    #define IIC_SDA_READ()  ((GPIO_PORT_IIC->IDR & IIC_SDA_PIN) != 0)	                    //读SDA线状态
#endif

/**************************************************************************************************/
//function
/**
 *	@brief IIC总线位延迟，最快400KHz
 *	@param None
 *	@return None
 */
static void IIC_Delay(void)
{
    uint8_t i;

    /**　
     	下面的时间是通过安富莱AX-Pro逻辑分析仪测试得到的。
    	CPU主频72MHz时，在内部Flash运行, MDK工程不优化
    	循环次数为10时，SCL频率 = 205KHz
    	循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us
     	循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us

    IAR工程编译效率高，不能设置为7
    */
    for (i = 0; i < 10; i++);
}

/**
 *	@brief CPU发起IIC总线启动信号
 *	@param None
 *	@return None
 */
void IIC_Start(void)
{
    //当SCL高电平时，SDA出现一个下跳沿表示IIC总线启动信号
    IIC_SDA_1();
    IIC_SCL_1();
    IIC_Delay();
    IIC_SDA_0();
    IIC_Delay();
    IIC_SCL_0();
    IIC_Delay();
}

/**
 *	@brief CPU发起IIC总线停止信号
 *	@param None
 *	@return None
 */
void IIC_Stop(void)
{
    //当SCL高电平时，SDA出现一个上跳沿表示IIC总线停止信号
    IIC_SDA_0();
    IIC_SCL_1();
    IIC_Delay();
    IIC_SDA_1();
}

/**
 *	@brief CPU向IIC总线设备发送8bit数据
 *	@param _ucByte[uint8_t] 等待发送的字节
 *	@return None
 */
void IIC_Send_Byte(uint8_t _ucByte)
{
    uint8_t i;

    //先发送字节的高位bit7
    for (i = 0; i < 8; i++)
    {
        if (_ucByte & 0x80)
            IIC_SDA_1();
        else
            IIC_SDA_0();

        IIC_Delay();
        IIC_SCL_1();
        IIC_Delay();
        IIC_SCL_0();
        if (i == 7)
            IIC_SDA_1(); //释放总线

        _ucByte <<= 1;	//左移一个bit
        IIC_Delay();
    }
}

/**
 *	@brief CPU从IIC总线设备读取8bit数据
 *	@param ack[uint8_t] 是否进行ack应答
 *	@return 读到的数据
 */
uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i;
    uint8_t value;

    //读到第1个bit为数据的bit7
    value = 0;
    for (i = 0; i < 8; i++)
    {
        value <<= 1;
        IIC_SCL_1();
        IIC_Delay();

        if (IIC_SDA_READ()) value++;

        IIC_SCL_0();
        IIC_Delay();
    }
    if(ack==0)
        IIC_NAck();
    else
        IIC_Ack();
    return value;
}

/**
 *	@brief CPU产生一个时钟，并读取器件的ACK应答信号
 *	@param None
 *	@return 返回0表示正确应答，1表示None器件响应
 */
uint8_t IIC_Wait_Ack(void)
{
    uint8_t re;

    IIC_SDA_1();	///CPU释放SDA总线
    IIC_Delay();
    IIC_SCL_1();	//CPU驱动SCL = 1, 此时器件会返回ACK应答
    IIC_Delay();

    if (IIC_SDA_READ())	//CPU读取SDA口线状态
        re = 1;
    else
        re = 0;

    IIC_SCL_0();
    IIC_Delay();
    return re;
}

/**
 *	@brief CPU产生一个ACK信号
 *	@param None
 *	@return None
 */
void IIC_Ack(void)
{
    IIC_SDA_0();	//CPU驱动SDA = 0
    IIC_Delay();
    IIC_SCL_1();	//CPU产生1个时钟
    IIC_Delay();
    IIC_SCL_0();
    IIC_Delay();
    IIC_SDA_1();	//CPU释放SDA总线
}

/**
 *	@brief CPU产生1个NACK信号
 *	@param None
 *	@return None
 */
void IIC_NAck(void)
{
    IIC_SDA_1();	//CPU驱动SDA = 1
    IIC_Delay();
    IIC_SCL_1();	//CPU产生1个时钟
    IIC_Delay();
    IIC_SCL_0();
    IIC_Delay();
}

/**
 *	@brief 配置IIC总线的GPIO，采用模拟IO的方式实现
 *	@param None
 *	@return None
 */
void IIC_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_IIC_ENABLE;	//打开GPIO时钟

    GPIO_InitStructure.Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;  	//开漏输出
    HAL_GPIO_Init(GPIO_PORT_IIC, &GPIO_InitStructure);

    //给一个停止信号, 复位IIC总线上的所有设备到待机模式
    IIC_Stop();
}

/**
 *	@brief 检测IIC总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
 *	@param _Address 设备的IIC总线地址
 *	@return 返回值 0 表示正确， 返回1表示未探测到
 */
uint8_t IIC_CheckDevice(uint8_t _Address)
{
    uint8_t ucAck;

    IIC_GPIO_Init();		//配置GPIO

    IIC_Start();		//发送启动信号

    //发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传
    IIC_Send_Byte(_Address|IIC_WR);
    ucAck = IIC_Wait_Ack();	//检测设备的ACK应答

    IIC_Stop();			//发送停止信号

    return ucAck;
}
