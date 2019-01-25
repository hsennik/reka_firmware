from pyb import Pin, SPI

m_cs_pin  = Pin("P7", Pin.OUT_OD)

m_cs_pin.value(0)
time.sleep(1000)
m_cs_pin.value(1)
