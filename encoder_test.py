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
from operator import sub

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

target = 10
increment = 5

command = target * 7

a_star.motors(int(command), int(-command))
time.sleep(0.01)
encoders0 = a_star.read_encoders()
time.sleep(0.01)
t1 = time.monotonic()
t3 = t1

#at full speed, full batt, we are getting about 5500-6000 encoder ticks per second, per wheel.   So assume 5000 per sec per wheel practical maximum

integral = 0
delta1=[0,0]
delta=[0,0]
initialized = False
loop_counter = 0

while done==0:
  time.sleep(0.01-0.0015)
  ts1 = time.time()
  #a_star.leds(1,1,1)
  #buttons = a_star.read_buttons()
  #analog = a_star.read_analog()
  battery_millivolts = a_star.read_battery_millivolts()
  encoders1 = a_star.read_encoders()
  delta2 = tuple(map(sub,encoders1,encoders0))
  delta[0] = (delta2[0]+delta1[0])/2
  delta[1] = (delta2[1]+delta1[1])/2
  #delta = delta2
  delta1 = delta2
  encoders0 = encoders1
  #print(ts2-ts1,analog)  
  t2=time.monotonic()
  dT = t2-t1
  error = target - delta[0]
  integral = integral + error * 0.3
  command = (target * 7) + error * 5 + integral 
  print("dT=%8.4f  target=%4.1f encoders=%6d,%6d=%4.1f,%4.1f   %5.1f,%5.1f  batt=%2.1f V   errors=%d" % (dT, target, encoders1[0],encoders1[1], delta[0],delta[1] , error, command, float(battery_millivolts[0])/1000.0 , a_star.errors) )  
  t1 = t2
  a_star.motors(int(command), int(-command))
  if increment > 0 and target > 40: increment = -5
  if increment < 0 and target < 1: increment = 5
  if t2-t3 > 2:
    target = target + increment
    t3 = t2
  

a_star.motors(0, 0)

