#!/usr/bin/env python3

from a_star import AStar
import time
import signal
import sys
import threading
import math
import termios
import fcntl
import os
#from math import cos,sin,pi
#from pynput import keyboard

done = 0


def signal_handler(sig, frame):
  global done
  #print('You pressed Ctrl+C!')
  #a_star.motors(0, 0);
  #sys.exit(0)
  done=1


print("installing SIGINT handler")
signal.signal(signal.SIGINT, signal_handler)

a_star = AStar()


t_print=time.time()

  
a_star.motors(int(500), int(500))

#at full speed, full batt, we are getting about 6000 encoder ticks per second, per wheel.   So assume 5000 per sec per wheel practical maximum
  
while done==0:
  time.sleep(0.1)
  ts1 = time.time()
  #a_star.leds(1,1,1)
  #buttons = a_star.read_buttons()
  #analog = a_star.read_analog()
  battery_millivolts = a_star.read_battery_millivolts()
  encoders = a_star.read_encoders()
  #print(ts2-ts1,analog)  
  t=time.monotonic()
  print("T=%8.4f  encoders=%6d,%6d   batt=%2.1f V   errors=%d" % (t, encoders[0],encoders[1] ,float(battery_millivolts[0])/1000.0 , a_star.errors) )  
