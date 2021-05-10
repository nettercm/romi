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
import js_linux as js
import numpy as np

#from math import cos,sin,pi
#from pynput import keyboard

done = 0

ir_far_xp=[130,145,165,183,197,210,263,337,419,576,700,840]
ir_far_fp=[60,48,40,30,25,20,15,10,7.5,5,4,3]

ir_near_xp=[65,112,201,257,325,467,580,660,800]
ir_near_fp=[30,20,10,8,6,4,3,2.5,2]


def signal_handler(sig, frame):
  global done
  print('You pressed Ctrl+C!')
  done=1
  time.sleep(0.1)
  js.js_deinit()
  a_star.motors(0, 0)
  sys.exit(0)


print("installing SIGINT handler")
signal.signal(signal.SIGINT, signal_handler)

a_star = AStar()


#while 1:
#  a_star.leds(1,0,0)
#  time.sleep(0.0001)
#  a_star.leds(0,1,0)
#  time.sleep(0.0001)
#  a_star.leds(0,0,1)
#  time.sleep(0.0001)

ts0 = 0.0

t_print=time.time()


js.js_init()

max_speed = 400

t_last_print = time.monotonic()

while done==0:
  a_star.leds(1,1,1)
  buttons = a_star.read_buttons()
  analog = a_star.read_analog()
  battery_millivolts = a_star.read_battery_millivolts()
  encoders = a_star.read_encoders()

  ir_left  = np.interp(analog[3],ir_near_xp,ir_near_fp)*25.4
  ir_front = np.interp(analog[1],ir_far_xp,ir_far_fp)*25.4
  ir_right = np.interp(analog[2],ir_near_xp,ir_near_fp)*25.4
  t=time.monotonic()

  y = js.axis_states['y']
  ry = js.axis_states['ry']
  rx = js.axis_states['rx']
  l_speed = y*-max_speed
  r_speed = l_speed

  l_speed = l_speed + rx*max_speed
  r_speed = r_speed - rx*max_speed
  #print(y,rx,l_speed,r_speed)
  a_star.motors(int(l_speed), int(r_speed))

  if t > t_last_print + 0.1:
    print("T=%8.4f    %-4.2f  %-4.2f   ir(adc)= %4d,  %4d,  %4d   ir = %5.0f,  %5.0f,  %5.0f   us= %4d,  %4d      %2.1f V      e=%6d,%6d    errors=%d" % (t, js.axis_states['y'],js.axis_states['rx'],    analog[3]/1,  analog[1]/1,  analog[2]/1,   ir_left, ir_front, ir_right,  analog[0]/1,  analog[4]/1,float(battery_millivolts[0])/1000.0, encoders[0], encoders[1], a_star.errors) )  
    #print("%-4.2f  %-4.2f" % (js.axis_states['y'],js.axis_states['ry']) )
    t_last_print = t

  time.sleep(0.01)  
