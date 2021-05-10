"""
from __future__ import print_function
from __future__ import absolute_import
from __future__ import division
from __future__ import unicode_literals
from builtins import bytes
from builtins import range
from future import standard_library
standard_library.install_aliases()
from builtins import object
"""

import smbus
import struct
import time

class AStar(object):
  def __init__(self):
    self.bus = smbus.SMBus(1)

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

    time.sleep(0.0001)
    try:
      byte_list = [self.bus.read_byte(20) for _ in range(size)]
    except:
      print ("Error 2")
      byte_list = [0]
      return 0
    else:
      #print(format,byte_list, bytes(byte_list));
      return struct.unpack(format, bytes(byte_list))

  def write_pack(self, address, format, *data):
    data_array = list(struct.pack(format, *data))
    #time.sleep(0.0001)
    try:
      print(format, data_array);
      #self.bus.write_i2c_block_data(20, address, data_array)
    except:
      print ("Error 3")

    #time.sleep(0.0001)

  def leds(self, red, yellow, green):
    self.write_pack(0, 'BBB', red, yellow, green)

  def play_notes(self, notes):
    self.write_pack(24, 'B15s', 1, notes.encode("ascii"))

  def motors(self, left, right):
    self.write_pack(6, 'hh', left, right)

  def read_buttons(self):
    return self.read_unpack(3, 3, "???")

  def read_battery_millivolts(self):
    return self.read_unpack(10, 2, "H")

  def read_analog(self):
    return self.read_unpack(12, 12, "HHHHHH")

  def read_encoders(self):
    return self.read_unpack(39, 4, 'hh')

  def test_read8(self):
    self.read_unpack(0, 8, 'cccccccc')

  def test_read32(self):
    self.read_unpack(0, 32, 'cccccccccccccccccccccccccccccccc')

  def test_write8(self):
    self.bus.write_i2c_block_data(20, 0, [0,0,0,0,0,0,0,0])
    time.sleep(0.0002)
