/**
 * @file    CCD_TSL1401.h
 * @brief   CCD_TSL1401 头文件
 * @details 包含TSL1401所有的预定义和函数声明
 * @version V4.0
 * @date    2023/7/16
 * @note    记得把这句定义加到main.h里面去
 *          #include "CCD.h"
 *          extern ADC_HandleTypeDef hadc1;
 */

#ifndef __CCD_TSL1401_H
#define __CCD_TSL1401_H

/************************************************************************/
//Private define
#define CCD_AO_Pin          GPIO_PIN_0
#define CCD_AO_GPIO_Port    GPIOC
#define CCD_SI_Pin          GPIO_PIN_1
#define CCD_SI_GPIO_Port    GPIOC
#define CCD_CLK_Pin         GPIO_PIN_2
#define CCD_CLK_GPIO_Port   GPIOC

//以下定义请复制到main.h中
#define CCD_AO_CHANNEL      ADC_CHANNEL_0
#define CCD_AO_hadc         hadc1           //PC0 ADC1_IN10

#define CCD_CLK_HIGH()      HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET)
#define CCD_CLK_LOW()       HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_RESET)

#define CCD_SI_HIGH()       HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_SET)
#define CCD_SI_LOW()        HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET)

#define CCD_DELAY(us)       Delay_us(us)    //Software delay

/*************************************************************************/
//struct
typedef struct CCD_TSL1401
{
    uint32_t CCD_Value[128];
    uint32_t Threshold;
    uint8_t  Position[3];

    /**
    * @brief   发出CCD读入信号
    * @param   None
    * @return  None
    * @note    None
    */
    void     (*StartRead)(void);

    /**
    * @brief   读取CCD数据
    * @param   CCD_Value[uint32_t*] 指向CCD数据存储区的指针
    * @return  None
    * @note    None
    */
    void     (*Read)(uint32_t*);

    /**
    * @brief   处理CCD数据，有些CCD前15位数据是0导致最后结果有误
    * @param   CCD_Value[uint32_t*] 指向CCD数据存储区的指针
    * @return  None
    * @note    None
    */
    void     (*DataTransform)(uint32_t*);

    /**
    * @brief   计算CCD读得的数据的阈值
    * @param   CCD_Value[uint32_t*] 指向CCD数据存储区的指针
    * @return  Threshold[uint32_t]: 阈值
    * @note    None
    */
    uint32_t (*CalcuThreshold)(uint32_t*);

    /**
    * @brief   根据阈值进行数据二值化
    * @param   CCD_Value[uint32_t*] 指向CCD数据存储区的指针
    * @param   Threshold[uint32_t]: 阈值
    * @return  None
    * @note    None
    */
    void     (*Binarization)(uint32_t*, uint32_t);

    /**
    * @brief   计算轨道左右边界的位置
    * @param   CCD_Value[uint32_t*] 指向CCD数据存储区的指针
    * @return  None
    * @note    None
    */
    void     (*CalcuPosition)(uint32_t*);

    /**
    * @brief   清理数据存储区
    * @param   None
    * @return  None
    * @note    None
    */
    void     (*ValueClear)(uint32_t*);
}CCD_TSL1401_Handler;

/*************************************************************************/
//Private function
void     Delay_us(uint8_t us);
void     TSL1401_Init(void);
void     TSL1401_ADC_Init(void);
void     TSL1401_StartRead(void);
void     TSL1401_Read(uint32_t *CCD_Value);
void     TSL1401_Data_Transform(uint32_t *CCD_Value);  
uint32_t TSL1401_CalcuThreshold(uint32_t *CCD_Value);
void     TSL1401_Binarization(uint32_t *CCD_Value, uint32_t Threshold);
void     TSL1401_CalcuPosition(uint32_t *CCD_Value);
void     TSL1401_Value_Clear(uint32_t *CCD_Value);

#endif /* __CCD_TSL1401_H */
