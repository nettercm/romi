#!/usr/bin/env python3

from smbus2 import *
import struct
import time

b = SMBus(1)
romi_msg = i2c_msg.read(20,4)
qtpy_msg = i2c_msg.read(8,8)

t_next = time.monotonic() + 0.01

while True:
     t = time.monotonic()
     if t_next > t:
          time.sleep(t_next - t - 0.00003)
     t = time.monotonic()
     t_next = t + 0.01
     print("%8.3f " % (t),end='')

     #send a command byte to the romi
     #cmd=39 means read encoders with the next read transaction
     b.write_byte(20,39) 

     #can't immediately perform a read from the romi since the 32u4 is not fast enough
     
     #read a packet from the qtpy
     b.i2c_rdwr(qtpy_msg)
     data2=list(qtpy_msg)

     #now read the data from the romi
     b.i2c_rdwr(romi_msg)
     data1=list(romi_msg)

     print(data1,data2)


