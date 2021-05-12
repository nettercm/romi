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


ts0 = 0.0

t_print=time.time()

  
  
while done==0:
  time.sleep(0.0068)
  ts1 = time.time()
  dT = ts1 - ts0
  ts0 = ts1
  a_star.leds(1,1,1)
  buttons = a_star.read_buttons()
  analog = a_star.read_analog()
  battery_millivolts = a_star.read_battery_millivolts()
  encoders = a_star.read_encoders()
  a_star.motors(int(0), int(0))
  ts2 = time.time()
  #print(ts2-ts1,analog)  
  t=time.monotonic()
  print("T=%8.4f  dT=%6.3f  %1.2f  %4d,%4d,%4d,%4d,%4d,%4d   %2.1f V   %3.2f V/cell  e=%6d,%6d  errors=%d" % (t, dT, (ts2-ts1)*1000,  analog[3]/1, analog[1]/1, analog[5]/1,  analog[0]/1, analog[2]/1, analog[4]/1,float(battery_millivolts[0])/1000.0,float(battery_millivolts[0])/6000.0 , encoders[0],encoders[1], a_star.errors) )  

  time.sleep(0.0069)
  ts1 = time.time()
  a_star.leds(0,0,0)
  buttons = a_star.read_buttons()
  analog = a_star.read_analog()
  battery_millivolts = a_star.read_battery_millivolts()
  encoders = a_star.read_encoders()
  a_star.motors(int(0), int(0))

  ts2 = time.time()
  #print("%1.2f" % ((ts2-ts1)*1000))  
  #print(battery_millivolts)
