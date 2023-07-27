/**
 * @file    CCD_TSL1401.c
 * @brief   CCD_TSL1401 C文件
 * @details 包含TSL1401所有的函数定义和全局变量
 * @version V4.0
 * @date    2023/7/16
 */

/*************************************************************************/
//Include
#include "CCD.h"

/*************************************************************************/
//Private Variable
struct CCD ccd;
uint8_t CCD_Struct_Init_Flag = 0;

/*************************************************************************/
//Private function
/**
    * @brief   延时1us
    * @param   None
    * @return  None
    * @note    None
    */
void Delay_us(uint8_t us)
{ 
   for(int ii=0;ii<us;ii++);      
}

void TSL1401_Init(void)
{
    if(CCD_Struct_Init_Flag == 0)
    {
        for(int i=0; i<128; i++)
            ccd.tsl1401.CCD_Value[i] = 0;
        for(int i=0; i<3; i++)
            ccd.tsl1401.Position[i]  = 0;
        ccd.tsl1401.Threshold        = 0;
        ccd.tsl1401.StartRead        = TSL1401_StartRead;
        ccd.tsl1401.Read             = TSL1401_Read;
        ccd.tsl1401.DataTransform    = TSL1401_Data_Transform;
        ccd.tsl1401.CalcuThreshold   = TSL1401_CalcuThreshold;
        ccd.tsl1401.Binarization     = TSL1401_Binarization;
        ccd.tsl1401.CalcuPosition    = TSL1401_CalcuPosition;
        ccd.tsl1401.ValueClear       = TSL1401_Value_Clear;

        CCD_Struct_Init_Flag         = 1;

        HAL_ADCEx_Calibration_Start(&CCD_AO_hadc);  //校准ADC
    }
}

/**
    * @brief   CCD所用ADC初始化
    * @param   None
    * @return  None
    * @note    None
    */
void TSL1401_ADC_Init(void)
{
    HAL_ADC_Start(&CCD_AO_hadc);
    HAL_ADC_PollForConversion(&CCD_AO_hadc, 1);
}
/**
    * @brief   发出CCD读入信号
    * @param   None
    * @return  None
    * @note    None
    */
void TSL1401_StartRead(void)  
{
    if(CCD_Struct_Init_Flag == 0)
        TSL1401_Init();

    CCD_SI_LOW();
    CCD_CLK_HIGH();
    CCD_DELAY(10);

    CCD_CLK_LOW();
    CCD_SI_HIGH();
    CCD_DELAY(10);

    CCD_CLK_HIGH(); 
    CCD_SI_LOW();
    CCD_DELAY(10);
}

/**
    * @brief   读取CCD数据
    * @param   CCD_Value[uint32_t*] 指向CCD数据存储区的指针
    * @return  None
    * @note    None
    */
void TSL1401_Read(uint32_t *CCD_Value)
{
    TSL1401_StartRead();
    for(uint8_t i=0; i<128; i++)
    {
        CCD_CLK_LOW();
        CCD_DELAY(20);
        TSL1401_ADC_Init();
        CCD_Value[i] = (HAL_ADC_GetValue(&CCD_AO_hadc)) >> 4;
        CCD_CLK_HIGH();
        CCD_DELAY(10);
    }
}

/**
    * @brief   处理CCD数据，有些CCD前15位数据是0导致最后结果有误
    * @param   CCD_Value[uint32_t*] 指向CCD数据存储区的指针
    * @return  None
    * @note    None
    */
void TSL1401_Data_Transform(uint32_t *CCD_Value)
{
    for(int i=0; i<(128-16); i++)
        CCD_Value[i] = CCD_Value[i+16];
    for(int i=(128-16); i<128; i++)
        CCD_Value[i] = CCD_Value[128-17];
    
}

/**
    * @brief   计算CCD读得的数据的阈值
    * @param   CCD_Value[uint32_t*] 指向CCD数据存储区的指针
    * @return  Threshold[uint32_t]: 阈值
    * @note    None
    */
uint32_t TSL1401_CalcuThreshold(uint32_t *CCD_Value)
{
    uint32_t Max=CCD_Value[0], Min=CCD_Value[0];

    for(uint8_t i=0; i<128; i++)
    {
        if(CCD_Value[i] > Max)
        {
            Max = CCD_Value[i];
        }
        if(CCD_Value[i] < Min)
        {
            Min = CCD_Value[i];
        }
    }

    return (Max+Min)/2;
}

/**
    * @brief   根据阈值进行数据二值化
    * @param   CCD_Value[uint32_t*] 指向CCD数据存储区的指针
    * @param   Threshold[uint32_t]: 阈值
    * @return  None
    * @note    None
    */
void TSL1401_Binarization(uint32_t *CCD_Value, uint32_t Threshold)
{
    for(uint8_t i=0; i<128; i++)
    {
        if(CCD_Value[i] > Threshold)
        {
            CCD_Value[i] = 1;
        }
        else
        {
            CCD_Value[i] = 0;
        }
    }
}

/**
    * @brief   计算轨道左右边界的位置
    * @param   CCD_Value[uint32_t*] 指向CCD数据存储区的指针
    * @return  None
    * @note    None
    */
void TSL1401_CalcuPosition(uint32_t *CCD_Value)
{
    uint8_t Boundry = 0;
    ccd.tsl1401.Position[0] = 0;
    ccd.tsl1401.Position[1] = 0;
    ccd.tsl1401.Position[2] = 0;

    for(uint8_t i=0; i<128; i++)
        if(CCD_Value[i] == 1 && Boundry == 0)
        {
            ccd.tsl1401.Position[0] = i;
        }
        else if(CCD_Value[i] == 0)
            Boundry = 1;
        else if(CCD_Value[i] == 1 && Boundry == 1)
        {
            ccd.tsl1401.Position[1] = i;
            break;
        }
    
    ccd.tsl1401.Position[2] = Boundry;
}

/**
    * @brief   清理数据存储区
    * @param   None
    * @return  None
    * @note    None
    */
void TSL1401_Value_Clear(uint32_t *CCD_Value)
{
  for(int i=0; i<128; i++)
  {
    CCD_Value[i] = 0;
  }
}
