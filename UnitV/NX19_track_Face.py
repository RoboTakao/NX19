# track_Face - By: RoboTakao - 月 6月 29 2020

import sensor
import image
import lcd
import time
import utime
import KPU as kpu
from machine import UART
from Maix import GPIO
from fpioa_manager import *

fm.register(34,fm.fpioa.UART1_TX)
fm.register(35,fm.fpioa.UART1_RX)
uart_out = UART(UART.UART1, 38400, 8, None, 1, timeout=1000, read_buf_len=4096)

sensor.reset()
sensor.set_hmirror(1)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.run(1)

while False:
    uart_out.write('TEST\n')
    utime.sleep_ms(100)

task = kpu.load(0x300000)
anchor = (1.889, 2.5245, 2.9465, 3.94056, 3.99987, 5.3658, 5.155437, 6.92275, 6.718375, 9.01025)
a = kpu.init_yolo2(task, 0.5, 0.3, 5, anchor)

while True:
    img=sensor.snapshot()

    code = kpu.run_yolo2(task, img)
    if code:
        max_area = 0
        target = code[0]
        for i in code:
            if i.w()*i.h() > max_area:
                max_area = i.w()*i.h()
                target = i
        if uart_out.read(4096):
            print("OK")
            area = target.w()*target.h()
            tcx = int(target.x()+target.w()/2)
            tcy = int(target.y()+target.h()/2)
            hexlist = [(tcx >> 8) & 0xFF, tcx & 0xFF, (tcy >> 8) & 0xFF, tcy & 0xFF]
            uart_out.write(bytes(hexlist))
            print(tcx)
            print(tcy)
            print(hexlist)
            tmp=img.draw_rectangle(target.rect())
            tmp=img.draw_cross(tcx, tcy)
            c=img.get_pixel(tcx, tcy)
        else:
            pass

    else:
        if uart_out.read(4096):
            hexlist = [0x00, 0xA0, 0x00, 0x78]
            uart_out.write(bytes(hexlist))
        else:
            pass

    utime.sleep_ms(400)
