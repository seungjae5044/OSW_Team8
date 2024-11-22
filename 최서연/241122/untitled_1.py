# Untitled - By: sy040 - Fri Nov 22 2024
from pyb import UART
import sensor
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

uart = UART(3, 115200, timeout_char=200)

while True:
    uart.write("Hello World!\r")
    time.sleep_ms(1000)
    print('send')
