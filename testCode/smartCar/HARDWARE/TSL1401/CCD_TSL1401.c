/*
 * @file    CCD_TSL1401.c
 * @brief   CCD_TSL1401 function
 * @details This file including all functions needed for CCD_TSL1401
 * @version V2.0
 * @date    2023/7/4
 */

/************************************************************************/
//Include
#include "CCD_TSL1401.h"

/************************************************************************/
//Private Variable
uint8_t Position[3];

/************************************************************************/
//Private function
/*
    * @brief   Delay 1us
    * @details This function will delay 1us
    * @param   None
    * @return  None
    * @note    None
    */
void Delay_us(uint8_t us)
{ 
   for(int ii=0;ii<us;ii++);      
}

/*
    * @brief   CCD ADC Init
    * @details This function will init CCD ADC
    * @param   None
    * @return  None
    * @note    None
    */
void CCD_ADC_Init(void)
{
    HAL_ADC_Start(&CCD_AO_hadc);
    HAL_ADC_PollForConversion(&CCD_AO_hadc, 1);
}
/*
    * @brief   Start read CCD
    * @details This function will start read CCD
    * @param   None
    * @return  None
    * @note    None
    */
void CCD_StartCollection(void)  
{
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

/*
    * @brief   Read CCD
    * @details This function will read CCD value
    * @param   CCD_Value[uint32_t *]: A pointer to store CCD value
    * @return  None
    * @note    None
    */
void CCD_Read(uint32_t *CCD_Value)
{
    CCD_StartCollection();
    for(uint8_t i=0; i<128; i++)
    {
        CCD_CLK_LOW();
        CCD_DELAY(20);
        CCD_ADC_Init();
        CCD_Value[i] = (HAL_ADC_GetValue(&CCD_AO_hadc)) >> 4;
        CCD_CLK_HIGH();
        CCD_DELAY(10);
    }
}

/*
    * @brief   CCD data transform
    * @details This function will transform CCD value
    * @param   CCD_Value[uint32_t *]: A pointer to store CCD value
    * @return  None
    * @note    None
    */
void CCD_Data_Transform(uint32_t *CCD_Value)
{
    for(int i=0; i<(128-16); i++)
        CCD_Value[i] = CCD_Value[i+16];
    for(int i=(128-16); i<128; i++)
        CCD_Value[i] = CCD_Value[128-17];
    
}

/*
    * @brief   Calculate threshold
    * @details This function will calculate threshold of CCD value
    * @param   CCD_Value[uint32_t *]: A pointer to store CCD value
    * @return  Threshold[uint32_t]: Threshold value
    * @note    None
    */
uint32_t CCD_CalcuThreshold(uint32_t *CCD_Value)
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

/*
    * @brief   Binarization
    * @details This function will binarizate CCD value
    * @param   CCD_Value[uint32_t *]: A pointer to store CCD value
    * @param   Threshold[uint32_t]: Threshold value
    * @return  None
    * @note    None
    */
void CCD_Binarization(uint32_t *CCD_Value, uint32_t Threshold)
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

/*
    * @brief   Calculate position
    * @details This function will calculate position of black line
    * @param   CCD_Value[uint32_t *]: A pointer to store CCD value
    * @return  None
    * @note    None
    */
void CCD_CalcuPosition(uint32_t *CCD_Value)
{
    uint8_t Boundry = 0;
    Position[0] = 0;
    Position[1] = 0;
    Position[2] = 0;

    for(uint8_t i=0; i<128; i++)
        if(CCD_Value[i] == 1 && Boundry == 0)
        {
            Position[0] = i;
        }
        else if(CCD_Value[i] == 0)
            Boundry = 1;
        else if(CCD_Value[i] == 1 && Boundry == 1)
        {
            Position[1] = i;
            break;
        }
    
    Position[2] = Boundry;
}

/*
    * @brief   Clear CCD value
    * @details This function will clear CCD value
    * @param   None
    * @return  None
    * @note    None
    */
void CCD_Value_Clear(uint32_t *CCD_Value)
{
  for(int i=0; i<128; i++)
  {
    CCD_Value[i] = 0;
  }
}
