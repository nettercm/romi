# Copyright Pololu Corporation.  For more information, see https://www.pololu.com/

import smbus
import struct
import time

class AStar:

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

    self.error=0
    
    try:
      self.bus.write_byte(20, address)
    except:
      #print ("Error 1")
      self.errors+=1
      self.error=1

    time.sleep(0.0001)

    try:
      byte_list = [self.bus.read_byte(20) for _ in range(size)]
    except:
      #print ("Error 2")
      byte_list = [0]
      self.errors+=1
      self.error=1
      return 0
    else:
      return struct.unpack(format, bytes(byte_list))



  def write_pack(self, address, format, *data):
    data_array = list(struct.pack(format, *data))
    self.error=0
    try:
      self.bus.write_i2c_block_data(20, address, data_array)
    except:
      #print ("Error 3")
      self.errors+=1
      self.error=1



  def leds(self, red, yellow, green):
    self.error=0
    self.write_pack(0, 'BBB', red, yellow, green)
    if self.error:
      #print ("leds() failed")
      pass



  def play_notes(self, notes):
    self.error=0
    self.write_pack(24, 'B15s', 1, notes.encode("ascii"))
    if self.error:
      #print ("play_notes() failed")
      pass



  def motors(self, left, right):
    self.error=0
    self.write_pack(6, 'hh', left, right)
    if self.error:
      #print ("motors() failed")
      pass



  def read_buttons(self):
    self.error=0
    b = self.read_unpack(3, 3, "???")
    if self.error:
      #print ("read_buttons() failed")
      return [0,0,0]
    else:
      return b
    


  def read_battery_millivolts(self):
    self.error=0
    bmv = self.read_unpack(10, 2, "H")
    if self.error:
      #print ("read_battery_millivolts() failed")
      return [0]
    else:
      return bmv
      


  def read_analog(self):
    self.error=0
    a = self.read_unpack(12, 12, "HHHHHH")
    if self.error:
      #print ("read_analog() failed")
      return [0,0,0,0,0,0]
    else:
      return a



  def read_encoders(self):
    self.error=0
    e = self.read_unpack(39, 4, 'hh')
    if self.error:
      #print ("read_encoders failed")
      return [0,0]
    else:
      return e
    


  def test_read8(self):
    self.read_unpack(0, 8, 'cccccccc')



  def test_read32(self):
    self.read_unpack(0, 32, 'cccccccccccccccccccccccccccccccc')



  def test_write8(self):
    self.bus.write_i2c_block_data(20, 0, [0,0,0,0,0,0,0,0])
    time.sleep(0.0001)
