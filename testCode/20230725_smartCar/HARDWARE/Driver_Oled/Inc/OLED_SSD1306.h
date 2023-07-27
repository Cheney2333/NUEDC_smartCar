/**
 * @file    OLED_SSD1306.h
 * @brief   OLED_SSD1306 header file
 * @details This file including all define and function prototypes
 *         of OLED_SSD1306.c
 * @version V4.1
 * @date    2023/7/19
 * @usage   SSD1306_Init(&hi2cx);   //初始化OLED，hi2cx为你要使用的I2C句柄
 *          oled.ssd1306.xxx(...)   //xxx为结构体中定义函数指针，...为所需参数
 */
#ifndef __OLED_SSD1306_H
#define __OLED_SSD1306_H
/***********************************************************************/
//define
#define LEFT            0x27    //左移命令
#define RIGHT           0x26    //右移命令
#define RIGHT_UP        0x29    //右上移命令
#define LEFT_UP         0x2A    //左上移命令
#define InverseON       0xA7    //反色显示
#define InverseOFF      0xA6    //正常显示
#define SSD1306_CMD     0x00    //写命令
#define SSD1306_DATA    0x40    //写数据
#define SSD1306_EnXLen  8       //每个字节的列数
#define SSD1306_CnXLen  16      //每个汉字的列数s

/***********************************************************************/
//struct
typedef struct OLED_SSD1306
{
    I2C_HandleTypeDef *hi2c;
    uint8_t           Address;

    /**
     * @brief   开启OLED显示 
     * @param   None
     * @return  None
     * @note    None
     * @attention   None
     */
    void    (*On)(void);

    /**
     * @brief   关闭OLED显示 
     * @param   None
     * @return  None
     * @note    None
     * @attention   None
     */  
    void    (*Off)(void);

    /**
     * @brief   OLED清屏函数 
     * @param   None
     * @return  None
     * @note    None
     * @attention   None
     */
    void    (*Clear)(void);

    /**
     * @brief   OLED清屏函数 
     * @param   x[unsignedchar] 横坐标
     * @param	y[unsignedchar] 纵坐标
     * @return  None
     * @note    None
     * @attention   None
     */
    void    (*SetPos)(unsigned char x, unsigned char y);

    /**
     * @brief   OLED显示一个字符,包括部分字符 
     * @param   x[uint8_t] 横坐标
     * @param	y[uint8_t] 纵坐标
     * @param	chr[uint8_t] 要显示的字符
     * @param	Char_Size[uint8_t] 字体大小选择 16/12
     * @return  None
     * @note    None
     * @attention   None
     */
    void    (*ShowChar)(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size);

    /**
     * @brief   OLED显示一个字符串 
     * @param   x[uint8_t] 横坐标
     * @param	y[uint8_t] 纵坐标
     * @param	chr[uint8_t*] 要显示的字符串
     * @param	Char_Size[uint8_t] 字体大小选择 16/12
     * @return  None
     * @note    None
     * @attention   None
     */
    void    (*ShowString)(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size);
    
    /**
     * @brief   OLED显示汉字 
     * @param   x[uint8_t] 横坐标
     * @param	y[uint8_t] 纵坐标
     * @param	no[uint8_t] 要显示的中文字符序号
     * @return  None
     * @note    None
     * @attention   None
     */
    void    (*ShowCHinese)(uint8_t x,uint8_t y,uint8_t no);
    
    /**
     * @brief: 在OLED特定区域显示BMP图片
     * @param  x0[uint8_t] 图像开始显示横坐标  x0:0~127
     * @param  y0[uint8_t] 图像开始显示纵坐标  y0:0~7
     * @param  x1[uint8_t] 图像结束显示横坐标  x1:1~128
     * @param  y1[uint8_t] 图像结束显示纵坐标  y1:1~8
     * @param  *BMP[uint8_t*] 待显示的图像数据
     * @param  Color[uint8_t] 是否反相显示(1反相、0不反相)
     * @return None
     */
    void    (*DrawBMP)(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *BMP, uint8_t Color_Turn);
    
    /**
     * @brief: 屏幕内容水平全屏滚动播放
     * @param  direction[uint8_t] LEFT(0x27) or RIGHT(0x26)
     * @return None
     */ 
    void    (*HoriShift)(uint8_t direction);
    
    /**
     * @brief: 屏幕部分内容水平滚动播放
     * @param  direction[uint8_t] LEFT(0x27) or RIGHT(0x26)
     * @param  start[uint8_t] 开始页地址  0x00~0x07
     * @param  end[uint8_t] 结束页地址  0x01~0x07
     * @return None
     */
    void    (*PartHoriShift)(uint8_t direction, uint8_t start, uint8_t end);
    
    /**
     * @brief: 屏幕内容垂直水平全屏滚动播放
     * @param  direction[uint8_t] LEFT_UP(0x2A) or RIGHT_UP(0x29)
     * @return None
     */
    void    (*VerAndHoriShift)(uint8_t direction);
    
  /**
     * @brief: 屏幕内容取反显示
     * @param  mode[uint8_t] InverseON(0xA7) or InverseOFF(0xA6)
     * @return None
     */  
    void    (*ColorInverse)(uint8_t mode);
    
     /**
     * @brief: 屏幕亮度调节
     * @param  intensity[uint8_t] 0x00~0xFF,RESET=0x7F
     * @return None
     */        
    void    (*IntensityControl)(uint8_t intensity);

    /**
    * @brief: 字符串写入缓冲字符串检查，防止OLED后面显示异常
    * @param  TempStr[uint8_t] 字符串指针
    * @param  MaxLen[uint8_t] 字符串最大长度
    * @return None
    */
    void    (*TempStrCheck)(uint8_t *TempStr, uint8_t MaxLen);
}OLED_SSD1306_Handler;

/***********************************************************************/
//OLED控制用函数
void SSD1306_Struct_Init(void);  //初始化OLED结构体(自动的)
void SSD1306_WR_CMD(uint8_t cmd);
void SSD1306_WR_DATA(uint8_t data);
void SSD1306_Init(I2C_HandleTypeDef *hi2c);
void SSD1306_Display_On(void);
void SSD1306_Display_Off(void);
void SSD1306_Clear(void);
void SSD1306_SetPos(unsigned char x, unsigned char y);
void SSD1306_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size);
void SSD1306_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size);
void SSD1306_ShowCHinese(uint8_t x,uint8_t y,uint8_t no);
void SSD1306_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *BMP, uint8_t Color_Turn);
void SSD1306_HoriShift(uint8_t direction);
void SSD1306_PartHoriShift(uint8_t direction, uint8_t start, uint8_t end);
void SSD1306_VerAndHoriShift(uint8_t direction);
void SSD1306_ColorInverse(uint8_t mode);
void SSD1306_IntensityControl(uint8_t intensity);
void SSD1306_TempStrCheck(uint8_t *TempStr, uint8_t MaxLen);

#endif
