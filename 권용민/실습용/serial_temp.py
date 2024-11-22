# Untitled - By: user - Fri Nov 22 2024

#port sensor
import time
from pyb import UART

clock = time.clock()

uart = UART(3,115200,timeout_char=200)

while True:
    uart.write("hello\r")
    time.sleep_ms(1000)
    print('send')
