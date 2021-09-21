#!/usr/bin/env python3

from smbus2 import *
import struct
import time

b = SMBus(1)
romi_msg = i2c_msg.read(20,4)
qtpy_msg = i2c_msg.read(8,8)

t_next = time.monotonic() + 0.01

data1 = None
data2 = None
number_of_romi_errors = 0
number_of_qtpy_errors = 0
retries = 5
done = False
romi_error_string=""

while True:
     t = time.monotonic()
     if t_next > t:
          time.sleep(t_next - t - 0.000035)
     t = time.monotonic()
     t_next = t + 0.01

     romi_error = 0

     #send a command byte to the romi
     #cmd=39 means read encoders with the next read transaction

     try:
          b.write_byte(20,39) 
     except:
          romi_error = romi_error | 1
          romi_error_string += "1"


     '''
     #can't immediately perform a read from the romi since the 32u4 is not fast enough
     #read a packet from the qtpy
     #but only use the data if there was no error
     try:
          b.i2c_rdwr(qtpy_msg)
     except:
          number_of_qtpy_errors = number_of_qtpy_errors + 1
     else:
          data2=list(qtpy_msg)
     '''

     time.sleep(0.0005)

     #now read the data from the romi
     try:
          b.i2c_rdwr(romi_msg)
     except: 
          romi_error = romi_error | 2
          romi_error_string += "2"

     time.sleep(0.0005)

     try:
          b.write_block_data(20,6,[0,0,0,0])
     except: 
          romi_error = romi_error | 4
          romi_error_string += "4"

     #ignore data from the romi if there was an error
     if not romi_error:
          data1=list(romi_msg)
     else:
          number_of_romi_errors = number_of_romi_errors + 1
          #print("")
          

     print("%8.3f " % (t),end='')
     print(data1,data2,number_of_romi_errors,romi_error_string)

     if romi_error:
          time.sleep(0.00003)
          #print("")
