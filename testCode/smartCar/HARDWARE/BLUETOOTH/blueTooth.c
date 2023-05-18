#include "bluetooth.h"

uint8_t USART2_RX_BUF[USART2_REC_LEN]; // 接收缓冲,最大USART_REC_LEN个字节.
uint16_t USART2_RX_STA = 0;            // 接收状态标记//bit15：接收完成标志，bit14~0：接收到的有效字节数目
uint8_t USART2_NewData;                // 当前串口中断接收的1个字节数据的缓存

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // 串口中断回调函数
{
    if (huart == &huart2)
    {
        if ((USART2_RX_STA & 0x8000) == 0) // 接收未完成
        {
            if (USART2_NewData == 0x5A) // 接收到了0x5A
            {
                USART2_RX_STA |= 0x8000; // 接收完成了，将USART2_RX_STA中的bit15（15位）置1
            }
            else
            {
                USART2_RX_BUF[USART2_RX_STA & 0X7FFF] = USART2_NewData; // 将收到的数据放入数组，
                USART2_RX_STA++;                                        // 数据长度计数加1
                if (USART2_RX_STA > (USART2_REC_LEN - 1))
                    USART2_RX_STA = 0; // 接收数据错误,重新开始接收
            }
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&USART2_NewData, 1); // 因为每执行完一次中断回调函数会将接收中断功能关闭，所以最后需要再开启接收中断
    }
}
