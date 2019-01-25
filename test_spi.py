# Untitled - By: ns - Sun Jan 6 2019

import sensor, image, time
from pyb import Pin, SPI
from pyb import ExtInt

#line = 5
#def callback1(line):
#    flag_spi = 1

m_cs_pin  = Pin("P7", Pin.OUT_OD)
print('it do be like that')

#print('flag0')

spi = SPI(2, SPI.MASTER, baudrate=int(1000000), polarity=0, phase=0)
#SPI.init(SPI.MASTER, baudrate=int(1000000), polarity=0, phase=0, bits=8, firstbit=SPI.MSB, ti=False, crc=None)

#flag_spi = lambda test: test+1
pin = Pin('P6')
flag_spi = 0
def callback(pin):
    #print('pin change',pin)
    global flag_spi
    flag_spi = 1

ext = ExtInt(Pin('P6'), ExtInt.IRQ_RISING, Pin.PULL_DOWN,callback)
#might be PULL_NONE

#print('flag1')


#print(spi)
#spi.init(SPI.MASTER, baudrate=4000000,polarity=0,phase=0,bits=8,firstbit=SPI.MSB,ti=False,crc=None)
#print('flag2')

#SPI.send()
#SPI.recv()


#clock = time.clock()

#SPI.init()
#spi.init(SPI.MASTER,baudrate=4000000,*,prescaler(),polarity=0,phase=0,bits=8,firstbit=SPI.MSB,ti=False,crc=None)



#print('flag3')

while (True):
    counter = 0
    #clock.tick()
    #print('it do be more like that')
    m_cs_pin.low()
    #while (counter<1000):
    #    pass
    time.sleep(100)
    spi.send('AT')
    #print('flagtest')
    m_cs_pin.high()

    #print(flag_spi)

    if flag_spi == 1:
        m_cs_pin.low()
        print(spi.recv(100))
        m_cs_pin.high()

        flag_spi = 0


    #time.sleep(2)

#intializations from HW SPI constructor
#_physical_transport = BLUEFRUIT_TRANSPORT_HWSPI

#ns - ver OD/PP
#
#m_irq_pin = Pin("P7", Pin.IN)
#m_rst_pin = Pin("P8", Pin.OUT_PP)

#m_miso_pin = m_mosi_pin = m_sck_pin = -1

#m_tx_count = 0

#m_mode_switch_command_enabled = true


#VERBOSE_MODE = true
#_verbose = VERBOSE_MODE
#ms_cs_pin.value

#m_cs_pin.value(1)


