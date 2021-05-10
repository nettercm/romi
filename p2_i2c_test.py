#!/usr/bin/env python2

import smbus
import struct
import time

class AStar(object):
  def __init__(self):
    self.bus = smbus.SMBus(1)
    self.errors = 0 #cumulative number of errors
    self.error = 0 #result of the last operation - gets reset at the beginning of the op

  def read_unpack(self, address, size, format):
    # Ideally we could do this:
    #    byte_list = self.bus.read_i2c_block_data(20, address, size)
    # But the AVR's TWI module can't handle a quick write->read transition,
    # since the STOP interrupt will occasionally happen after the START
    # condition, and the TWI module is disabled until the interrupt can
    # be processed.
    #
    # A delay of 0.0001 (100 us) after each write is enough to account
    # for the worst-case situation in our example code.

    #time.sleep(0.0001)
    try:
      self.bus.write_byte(20, address)
    except:
      print ("Error 1")
      self.errors+=1
      self.error=1

    time.sleep(0.0001)
    try:
      byte_list = [self.bus.read_byte(20) for _ in range(size)]
    except:
      print ("Error 2")
      byte_list = [0]
      self.errors+=1
      self.error=1
      return 0
    else:
      #print(format,byte_list, bytes(byte_list));
      return struct.unpack(format, bytes(byte_list))

  def write_pack(self, address, format, *data):
    data_array = list(struct.pack(format, *data))
    #time.sleep(0.0001)
    try:
      #print(data_array);
      self.bus.write_i2c_block_data(20, address, data_array)
    except:
      print ("Error 3")
      self.errors+=1
      self.error=1

    #time.sleep(0.0001)


  def leds(self, red, yellow, green):
    self.error=0
    try:
      self.bus.write_i2c_block_data(20, 0, [red,yellow,green])
    except:
      print ("leds(): Error 3")
      self.errors+=1
      self.error=1


  def play_notes(self, notes):
    #self.write_pack(24, 'B15s', 1, notes.encode("ascii"))
    return


  def motors(self, left, right):
    #self.write_pack(6, 'hh', left, right)
    self.error=0
    data=[left&0x00ff, (left&0xff00)>>8, right&0x00ff, (right&0xff00)>>8]
    try:
      self.bus.write_i2c_block_data(20, 6, data)
    except:
      print ("motors(): Error 3")
      self.errors+=1
      self.error=1
    return


  def read_buttons(self):
    self.error=0
    try:
      self.bus.write_byte(20, 3)
    except:
      print ("read_buttons(): Error 1")
      self.errors+=1
      self.error=1
      return [0,0,0]
    time.sleep(0.0001)
    try:
      byte_list = [self.bus.read_byte(20) for _ in range(3)]
    except:
      print ("read_buttons(): Error 2")
      byte_list = [0]
      self.errors+=1
      self.error=1
      return [0,0,0]
    else:
      return byte_list


  def read_battery_millivolts(self):
    self.error=0
    try:
      self.bus.write_byte(20, 10)
    except:
      print ("read_battery_millivolts(): Error 1")
      self.errors+=1
      self.error=1
      return 0
    time.sleep(0.0005)
    try:
      b = [self.bus.read_byte(20) for _ in range(2)]
    except:
      print ("read_battery_millivolts(): Error 2")
      self.errors+=1
      self.error=1
      return 0
    else:
      return b[0]+b[1]*256
      

  def read_analog(self):
    self.error=0
    try:
      self.bus.write_byte(20, 12)
    except:
      print ("read_analog(): Error 1")
      self.errors+=1
      self.error=1
      return [0,0,0,0,0,0]
    time.sleep(0.0001)
    try:
      b = [self.bus.read_byte(20) for _ in range(12)]
    except:
      print ("read_analog(): Error 2")
      self.errors+=1
      self.error=1
      return [0,0,0,0,0,0]
    else:
      return [b[0]+b[1]*256 , b[2]+b[3]*256 , b[4]+b[5]*256 , b[6]+b[7]*256 , b[8]+b[9]*256 , b[1]+b[11]*256 ] 


  def read_encoders(self):
    self.error=0
    try:
      self.bus.write_byte(20, 39)
    except:
      print ("read_encoders(): Error 1")
      self.errors+=1
      self.error=1
      return [0,0]
    time.sleep(0.0002)
    try:
      b = [self.bus.read_byte(20) for _ in range(4)]
    except:
      print ("read_encoders(): Error 2")
      self.errors+=1
      self.error=1
      return [0,0]
    else:
      return [b[0]+b[1]*256 , b[2]+b[3]*256 ] 


"""
A=AStar();

while True:
  t1=time.time()
  time.sleep(0.0002)
  t2=time.time()
  td=t2-t1
  if(td>0.00025):
    print(int((t2-t1)*1000000));

while True:
  t1=time.time();
  A.leds(1,1,1);
  b=A.read_buttons()
  e=A.read_encoders()
  a=A.read_analog()
  v=A.read_battery_millivolts()
  A.leds(0,0,0);
  t2=time.time();
  print(A.errors, b,e,a,v,int((t2-t1)*1000000));
  break;
"""
