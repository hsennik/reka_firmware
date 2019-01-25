# Untitled - By: ns - Sun Jan 6 2019

import sensor, image, time
from pyb import Pin, SPI

spi = SPI(2, SPI.MASTER, baudrate=int(4000000), polarity=0, phase=0)

#SPI.send()
#SPI.recv()


#intializations from HW SPI constructor
#_physical_transport = BLUEFRUIT_TRANSPORT_HWSPI

#ns - ver OD/PP
#m_cs_pin  = Pin("P6", Pin.OUT_OD)
#m_irq_pin = Pin("P7", Pin.IN)
#m_rst_pin = Pin("P8", Pin.OUT_PP)

#m_miso_pin = m_mosi_pin = m_sck_pin = -1

#m_tx_count = 0

#m_mode_switch_command_enabled = true


#VERBOSE_MODE = true
#_verbose = VERBOSE_MODE
#ms_cs_pin.value

#m_cs_pin.value(1)

SPI.init()
SPI.send("AT")
SPI.recv(receive)
print(receive)








#sensor.reset()
#sensor.set_pixformat(sensor.RGB565)
#sensor.set_framesize(sensor.QVGA)
#sensor.skip_frames(time = 2000)

clock = time.clock()

while(True):
    clock.tick()
#    img = sensor.snapshot()
    print(clock.fps())
