# OLED 驱动
## 一、OLED 驱动 简介
本驱动集合了所有我们见过的，我主要是怕很多OLED控制器不一样导致所用驱动不同，
就做了这一份文件，以防重复造轮子。

## 二、文件理论结构
OLED.h-OLED_SSD1306.h-OLED_SSD1306.c
      -...

## 三、使用方法
首先复制 Driver_Oled 文件夹到工程下的 Drivers 文件夹，然后用keil打开工程
在 Options for Target 选项卡下的 C/C++ 分页里的 Include Paths 添加
Driver_Oled 文件夹下的 Inc 文件夹。

然后在旁边的 Manage Project Items 选项卡下，在中间的 Groups 分页添加
Driver_Oled 文件夹，添加完毕后点击，在后侧添加  Driver_Oled 文件夹下的
Src 文件夹中的 OLED_xxx(你想用的型号).c

（以上都是作者我强迫症想这么整，你可以简单点直接把 Driver_Oled 文件夹、
Core 文件夹的 Inc 和 Src 文件夹合起来，然后在 Manage Project Items 选项卡下
添加 OLED_xxx(你想用的型号).c ）

在main.h文件里添加
> #include "OLED.h"
> extern I2C_HandleTypeDef hi2c1;   //适用于使用I2C协议进行通信的OLED屏幕

随后在main.c文件里添加
> SSD1306_Init();
即可使用驱动中定义的 oled 类进行OLED屏幕的操作。

## 四、oled 类操作方法
本驱动基于C++的类思想封装了OLED屏幕相关操作，基本调用结构如下：
> oled.型号.操作();

例如，对于使用SSD1306控制器的OLED屏幕，所有的操作如下
> oled.ssd1306.On();                                //打开OLED
> oled.ssd1306.Off();                               //关闭OLED
> oled.ssd1306.Clear();                             //清屏
> oled.ssd1306.SetPos(0,0);                         //设置光标位置
> oled.ssd1306.ShowChar(0,0,'A',16);                //显示字符
> oled.ssd1306.ShowString(0,0,"Hello World!",16);   //显示字符串
> oled.ssd1306.ShowCHinese(0,0,0);                  //显示汉字
> oled.ssd1306.VerScroll();                         //垂直滚动

具体的函数定义、传入参数、返回值等可以参考 OLED_SSD1306.c 文件里的函数
> SSD1306_Struct_Init()
找到实际函数对照，进行观看。（其实你自己实际使用一下oled类，vscode会自动
给你怎么写传入参数的提示）

