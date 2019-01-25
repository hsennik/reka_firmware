# Untitled - By: ns - Fri Jan 11 2019

import sensor, image, time
from pyb import UART

uart = UART(3, 9600, timeout_char=1000)
uart.init(9600, bits=8, parity=None, stop=1, timeout_char=1000)

#sensor.reset()
#sensor.set_pixformat(sensor.RGB565)
#sensor.set_framesize(sensor.QVGA)
#sensor.skip_frames(time = 2000)

clock = time.clock()

while(True):
    clock.tick()
#    img = sensor.snapshot()
#    print(clock.fps())
    print(uart.readline())
