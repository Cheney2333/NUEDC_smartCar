import sensor, image, lcd
import time
from machine import Timer
from fpioa_manager import fm
from machine import UART

#lcd初始化
lcd.init(freq=15000000)

#摄像头初始化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.run(1)
sensor.skip_frames(time = 2000)

# 配置串口
#fm.register(5, fm.fpioa.UART1_TX, force=True)
#fm.register(4, fm.fpioa.UART1_RX, force=True)
#uart = UART(UART.UART1, 115200, 8, 0, 1, timeout=1000, read_buf_len=4096)

thresholds = (9, 96, 4, 32, 5, 32)   # 红色阈值

while True:
    img = sensor.snapshot()

    blobs = img.find_blobs([thresholds])
    if blobs:
        for b in blobs:
            # 圈出色块
            tmp=img.draw_rectangle(b[0:4])
            tmp=img.draw_cross(b[5], b[6])
            # 打印色块中心点坐标
            print('x={},y={}'.format(b[5], b[6]))
            img.draw_string(0,0, 'x={},y={}'.format(b[5], b[6]))
            #uart.write('x={},y={}\r\n'.format(b[5], b[6]))


    lcd.rotation(2)
    lcd.display(img)     # LCD显示图片
