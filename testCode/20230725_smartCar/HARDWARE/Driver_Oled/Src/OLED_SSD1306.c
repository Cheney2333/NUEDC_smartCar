/*
 * @file    OLED_SSD1306.c
 * @brief   OLED_SSD1306 function
 * @details This file including all functions needed for OLED SSD1306
 * @version V4.1
 * @date    2023/7/19
 */

/********************************************************************/
//include
#include "OLED.h"

/********************************************************************/
//variable
struct OLED oled;
uint8_t oled_struct_init_flag = 0;

/********************************************************************/
//OLED控制用函数
/**
 * @brief   OLED结构体初始化
 * @param   None
 * @return  None
 * @note    None
 * @attention   None
 */
void SSD1306_Struct_Init(void)
{
	if(oled_struct_init_flag == 0)
	{
		oled.ssd1306.Address          = 0x78;
		oled.ssd1306.On               = SSD1306_Display_On;
		oled.ssd1306.Off              = SSD1306_Display_Off;
		oled.ssd1306.Clear            = SSD1306_Clear;
		oled.ssd1306.SetPos           = SSD1306_SetPos;
		oled.ssd1306.ShowChar         = SSD1306_ShowChar;
		oled.ssd1306.ShowString       = SSD1306_ShowString;
		oled.ssd1306.ShowCHinese      = SSD1306_ShowCHinese;
		oled.ssd1306.DrawBMP		  = SSD1306_DrawBMP;
		oled.ssd1306.HoriShift 		  = SSD1306_HoriShift;
		oled.ssd1306.PartHoriShift 	  = SSD1306_PartHoriShift;
		oled.ssd1306.VerAndHoriShift  = SSD1306_VerAndHoriShift;
		oled.ssd1306.ColorInverse	  = SSD1306_ColorInverse;
		oled.ssd1306.IntensityControl = SSD1306_IntensityControl;
		oled.ssd1306.TempStrCheck	  = SSD1306_TempStrCheck;
 		
		oled_struct_init_flag 		 = 1;
	}
}

/**
 * @brief   OLED写命令
 * @param   cmd[uint8_t] 命令
 * @return  None
 * @note    None
 * @attention   None
 */
void SSD1306_WR_CMD(uint8_t cmd)
{
	HAL_I2C_Mem_Write((oled.ssd1306.hi2c), oled.ssd1306.Address, SSD1306_CMD, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 0x100);
}

/**
 * @brief   OLED写数据
 * @param   data[uint8_t] 数据
 * @return  None
 * @note    None
 * @attention   None
 */
void SSD1306_WR_DATA(uint8_t data)
{
	HAL_I2C_Mem_Write((oled.ssd1306.hi2c), oled.ssd1306.Address, SSD1306_DATA, I2C_MEMADD_SIZE_8BIT, &data, 1, 0x100);
}

/**
 * @brief   OLED初始化
 * @param   hi2c[I2C_HandleTypeDef*] I2C句柄	
 * @return  None
 * @note    None
 * @attention   None
 */
void SSD1306_Init(I2C_HandleTypeDef *hi2c)
{
	if(oled_struct_init_flag == 0)
		SSD1306_Struct_Init();
	
	oled.ssd1306.hi2c = hi2c;

    HAL_Delay(1000); //这里的延时很重要
	
	SSD1306_WR_CMD(0xAE); //display off
	SSD1306_WR_CMD(0x20);	//Set Memory Addressing Mode	
	SSD1306_WR_CMD(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	SSD1306_WR_CMD(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	SSD1306_WR_CMD(0xc8);	//Set COM Output Scan Direction
	SSD1306_WR_CMD(0x00); //---set low column address
	SSD1306_WR_CMD(0x10); //---set high column address
	SSD1306_WR_CMD(0x40); //--set start line address
	SSD1306_WR_CMD(0x81); //--set contrast control register
	SSD1306_WR_CMD(0xff); //亮度调节 0x00~0xff
	SSD1306_WR_CMD(0xa1); //--set segment re-map 0 to 127
	SSD1306_WR_CMD(0xa6); //--set normal display
	SSD1306_WR_CMD(0xa8); //--set multiplex ratio(1 to 64)
	SSD1306_WR_CMD(0x3F); //
	SSD1306_WR_CMD(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	SSD1306_WR_CMD(0xd3); //-set display offset
	SSD1306_WR_CMD(0x00); //-not offset
	SSD1306_WR_CMD(0xd5); //--set display clock divide ratio/oscillator frequency
	SSD1306_WR_CMD(0xf0); //--set divide ratio
	SSD1306_WR_CMD(0xd9); //--set pre-charge period
	SSD1306_WR_CMD(0x22); //
	SSD1306_WR_CMD(0xda); //--set com pins hardware configuration
	SSD1306_WR_CMD(0x12);
	SSD1306_WR_CMD(0xdb); //--set vcomh
	SSD1306_WR_CMD(0x20); //0x20,0.77xVcc
	SSD1306_WR_CMD(0x8d); //--set DC-DC enable
	SSD1306_WR_CMD(0x14); //
	SSD1306_WR_CMD(0xaf); //--turn on oled panel
}
  
/**
 * @brief   开启OLED显示 
 * @param   None
 * @return  None
 * @note    None
 * @attention   None
 */
void SSD1306_Display_On(void)
{
	SSD1306_WR_CMD(0X8D);  //SET DCDC命令
	SSD1306_WR_CMD(0X14);  //DCDC ON
	SSD1306_WR_CMD(0XAF);  //DISPLAY ON
}

/**
 * @brief   关闭OLED显示 
 * @param   None
 * @return  None
 * @note    None
 * @attention   None
 */   
void SSD1306_Display_Off(void)
{
	SSD1306_WR_CMD(0X8D);  //SET DCDC命令
	SSD1306_WR_CMD(0X10);  //DCDC OFF
	SSD1306_WR_CMD(0XAE);  //DISPLAY OFF
}

/**
 * @brief   OLED清屏函数 
 * @param   None
 * @return  None
 * @note    None
 * @attention   None
 */
void SSD1306_Clear(void)
{
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		SSD1306_WR_CMD(0xb0+i);
		SSD1306_WR_CMD (0x00); 
		SSD1306_WR_CMD (0x10); 
		for(n=0;n<128;n++)
			SSD1306_WR_DATA(0x00);
	} 
}

/**
 * @brief   OLED清屏函数 
 * @param   x[unsignedchar] 横坐标
 * @param	y[unsignedchar] 纵坐标
 * @return  None
 * @note    None
 * @attention   None
 */
void SSD1306_SetPos(unsigned char x, unsigned char y)
{ 
	SSD1306_WR_CMD(0xb0+y);
	SSD1306_WR_CMD(((x&0xf0)>>4)|0x10);
	SSD1306_WR_CMD((x&0x0f)|0x01);
}

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
void SSD1306_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size)
{      	
	unsigned char c=0,i=0;	
	c=chr-' ';//得到偏移后的值			
	if(x>128-1){x=0;y=y+2;}
		if(Char_Size ==16)
			{
				SSD1306_SetPos(x,y);	
				for(i=0;i<8;i++)
					SSD1306_WR_DATA(F8X16[c*16+i]);
				SSD1306_SetPos(x,y+1);
				for(i=0;i<8;i++)
					SSD1306_WR_DATA(F8X16[c*16+i+8]);
			}
			else {	
				SSD1306_SetPos(x,y);
				for(i=0;i<6;i++)
					SSD1306_WR_DATA(F6x8[c][i]);
				
			}
}

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
void SSD1306_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		SSD1306_ShowChar(x,y,chr[j],Char_Size);
			x+=8;
		if(x>120){x=0;y+=2;}
			j++;
	}
}

/**
 * @brief   OLED显示汉字 
 * @param   x[uint8_t] 横坐标
 * @param	y[uint8_t] 纵坐标
 * @param	no[uint8_t] 要显示的中文字符序号
 * @return  None
 * @note    None
 * @attention   None
 */
void SSD1306_ShowCHinese(uint8_t x,uint8_t y,uint8_t no)
{      			    
	uint8_t t,adder=0;
	SSD1306_SetPos(x,y);	
    for(t=0;t<16;t++)
		{
				SSD1306_WR_DATA(Hzk[2*no][t]);		//Hzk 用取模软件得出的数组
				adder+=1;
     }	
		SSD1306_SetPos(x,y+1);	
    for(t=0;t<16;t++)
			{	
				SSD1306_WR_DATA(Hzk[2*no+1][t]);
				adder+=1;
      }					
}

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
void SSD1306_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *BMP, uint8_t Color)
{
	uint32_t j = 0;
	uint8_t x = 0, y = 0;

	if (y1 % 8 == 0)
		y = y1 / 8;
	else
		y = y1 / 8 + 1;
	for (y = y0; y < y1; y++)
	{
		SSD1306_SetPos(x0, y);
		for (x = x0; x < x1; x++)
		{
			if (Color)
				SSD1306_WR_DATA(~BMP[j++]); // 显示反相图片
			else
				SSD1306_WR_DATA(BMP[j++]); // 显示图片
		}
	}
}

/**
 * @brief: 屏幕内容水平全屏滚动播放
 * @param  direction[uint8_t] LEFT(0x27) or RIGHT(0x26)
 * @return None
 */
void SSD1306_HoriShift(uint8_t direction)

{
	SSD1306_WR_CMD(0x2e);		// 停止滚动
	SSD1306_WR_CMD(direction);  // 设置滚动方向
	SSD1306_WR_CMD(0x00);		// 虚拟字节设置，默认为0x00
	SSD1306_WR_CMD(0x00);		// 设置开始页地址
	SSD1306_WR_CMD(0x07);		// 设置每个滚动步骤之间的时间间隔的帧频
	//  0x00-5帧， 0x01-64帧， 0x02-128帧， 0x03-256帧， 0x04-3帧， 0x05-4帧， 0x06-25帧， 0x07-2帧，
	SSD1306_WR_CMD(0x07);   	// 设置结束页地址
	SSD1306_WR_CMD(0x00); 		// 虚拟字节设置，默认为0x00
	SSD1306_WR_CMD(0xff); 		// 虚拟字节设置，默认为0xff
	SSD1306_WR_CMD(0x2e); 		// 开启滚动-0x2f，禁用滚动-0x2e，禁用需要重写数据
}

/**
 * @brief: 屏幕部分内容水平滚动播放
 * @param  direction[uint8_t] LEFT(0x27) or RIGHT(0x26)
 * @param  start[uint8_t] 开始页地址  0x00~0x07
 * @param  end[uint8_t] 结束页地址  0x01~0x07
 * @return None
 */
void SSD1306_PartHoriShift(uint8_t direction, uint8_t start, uint8_t end)
{
	SSD1306_WR_CMD(0x2e);		// 停止滚动
	SSD1306_WR_CMD(direction);  // 设置滚动方向
	SSD1306_WR_CMD(0x00);		// 虚拟字节设置，默认为0x00
	SSD1306_WR_CMD(start);		// 设置开始页地址
	SSD1306_WR_CMD(0x07);		// 设置每个滚动步骤之间的时间间隔的帧频,0x07即滚动速度2帧
	SSD1306_WR_CMD(end);		// 设置结束页地址
	SSD1306_WR_CMD(0x00);		// 虚拟字节设置，默认为0x00
	SSD1306_WR_CMD(0xff);		// 虚拟字节设置，默认为0xff
	SSD1306_WR_CMD(0x2f);		// 开启滚动-0x2f，禁用滚动-0x2e，禁用需要重写数据
}

/**
 * @brief: 屏幕内容垂直水平全屏滚动播放
 * @param  direction[uint8_t] LEFT_UP(0x2A) or RIGHT_UP(0x29)
 * @return None
 */
void SSD1306_VerAndHoriShift(uint8_t direction)
{
	SSD1306_WR_CMD(0x2e);		// 停止滚动
	SSD1306_WR_CMD(direction);  // 设置滚动方向
	SSD1306_WR_CMD(0x01);		// 虚拟字节设置
	SSD1306_WR_CMD(0x00);		// 设置开始页地址
	SSD1306_WR_CMD(0x07);		// 设置每个滚动步骤之间的时间间隔的帧频，即滚动速度
	SSD1306_WR_CMD(0x07);		// 设置结束页地址
	SSD1306_WR_CMD(0x01);		// 垂直滚动偏移量
	SSD1306_WR_CMD(0x00);		// 虚拟字节设置，默认为0x00
	SSD1306_WR_CMD(0xff);		// 虚拟字节设置，默认为0xff
	SSD1306_WR_CMD(0x2f);		// 开启滚动-0x2f，禁用滚动-0x2e，禁用需要重写数据
}

/**
 * @brief: 屏幕内容取反显示
 * @param  mode[uint8_t] InverseON(0xA7) or InverseOFF(0xA6)
 * @return None
 */
void SSD1306_ColorInverse(uint8_t mode)
{
	SSD1306_WR_CMD(mode);
}

/**
 * @brief: 屏幕亮度调节
 * @param  intensity[uint8_t] 0x00~0xFF,RESET=0x7F
 * @return None
 */
void SSD1306_IntensityControl(uint8_t intensity)
{
	SSD1306_WR_CMD(0x81);
	SSD1306_WR_CMD(intensity);
}

/**
 * @brief: 字符串写入缓冲字符串检查，防止OLED后面显示异常
 * @param  TempStr[uint8_t] 字符串指针
 * @param  MaxLen[uint8_t] 字符串最大长度
 * @return None
 */
void SSD1306_TempStrCheck(uint8_t *TempStr, uint8_t MaxLen)
{
  uint8_t len = strlen((char *)TempStr);
  if(len < MaxLen) 
    for(uint8_t i = len; i < MaxLen; i++)
      TempStr[i] = ' ';
}
