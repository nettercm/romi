#!/usr/bin/env python3

# Copyright Pololu Corporation.  For more information, see https://www.pololu.com/
from a_star import AStar
import time
import signal
import sys
import threading
import math
#from math import cos,sin,pi
#from pynput import keyboard
import termios
import fcntl
import os

done = False

def signal_handler(sig, frame):
    global done
    print('qtr.py: You pressed Ctrl+C!')
    #a_star.motors(0, 0);
    done = True
    #sys.exit(0)


a_star = AStar()

line_crossing = 0
line_crossing_threashold = 70

a0_min=30
a0_max=650
a0_range=0.0
a0_k=1.0

a1_min=30
a1_max=650
a1_range=0.0
a1_k=1.0

a2_min=30
a2_max=650
a2_range=0.0
a2_k=1.0

a3_min=30
a3_max=650
a3_range=0.0
a3_k=1.0

a4_min=30
a4_max=650
a4_range=0.0
a4_k=1.0

a5_min=30
a5_max=650
a5_range=0.0
a5_k=1.0

adjust = 0
line = 0

lost_line = 0
found_line = 1

speed_cmd = 0

integral = 0
proportional = 0
derivative = 0
last_proportional = 0
lm = 0
rm = 0
slowdown_factor = 1.0
power_difference = 0

time_since_last_print = 0.0


"""
target_speed = 300
kp = 0.80
kd = 5.0
ki = 0.001
slowdown_threashold = 60
max_slowdown = 90
"""

#"""
target_speed = 110
kp = 0.42
kd = 1.0
ki = 0.000
slowdown_threashold = 90
max_slowdown=1
#"""

"""
target_speed = 210
kp = 0.58
kd = 2.5
ki = 0.0005
slowdown_threashold = 80
max_slowdown=10
"""


currently_crossing = False

def qtr_update(analog):

  global a0,a1,a2,a3,a4,a5
  global line,factor,found_line, lost_line
  global line_crossing
  global currently_crossing


  a0 = analog[0]
  if a0 < a0_min:
    a0 = a0_min
  if a0 > a0_max:
    a0 = a0_max

  a1 = analog[1]
  if a1 < a1_min:
    a1 = a1_min
  if a1 > a1_max:
    a1 = a1_max

  a2 = analog[2]
  if a2 < a2_min:
    a2 = a2_min
  if a2 > a2_max:
    a2 = a2_max

  a3 = analog[3]
  if a3 < a3_min:
    a3 = a3_min
  if a3 > a3_max:
    a3 = a3_max

  a4 = analog[4]
  if a4 < a4_min:
    a4 = a4_min
  if a4 > a4_max:
    a4 = a4_max

  a5 = analog[5]
  if a5 < a5_min:
    a5 = a5_min
  if a5 > a5_max:
    a5 = a5_max


  a0_range=float(a0_max-a0_min)
  a0_k=100.0/a0_range
  a0=100.0 - (float(a0) - float(a0_min) ) * a0_k

  a1_range=float(a1_max-a1_min)
  a1_k=100.0/a1_range
  a1=100.0 - (float(a1) - float(a1_min) ) * a1_k

  a2_range=float(a2_max-a2_min)
  a2_k=100.0/a2_range
  a2=100.0 - (float(a2) - float(a2_min) ) * a2_k

  a3_range=float(a3_max-a3_min)
  a3_k=100.0/a3_range
  a3=100.0 - (float(a3) - float(a3_min) ) * a3_k

  a4_range=float(a4_max-a4_min)
  a4_k=100.0/a4_range
  a4=100.0 - (float(a4) - float(a4_min) ) * a4_k

  a5_range=float(a5_max-a5_min)
  a5_k=100.0/a5_range
  a5=100.0 - (float(a5) - float(a5_min) ) * a5_k

  threashold_1 = 5.0

  if a0<threashold_1:
    a0=0.0
  if a1<threashold_1:
    a1=0.0
  if a2<threashold_1:
    a2=0.0
  if a3<threashold_1:
    a3=0.0
  if a4<threashold_1:
    a4=0.0
  if a5<threashold_1:
    a5=0.0
    
  a0 = 100.0 - a0
  a1 = 100.0 - a1
  a2 = 100.0 - a2
  a3 = 100.0 - a3
  a4 = 100.0 - a4
  a5 = 100.0 - a5
    
  avg = 0.0
  sum = 1.0
  
  avg = avg + 000.0 * a3
  sum = sum + a3
  
  avg = avg + 100.0 * a1
  sum = sum + a1
  
  avg = avg + 200.0 * a5
  sum = sum + a5
  
  avg = avg + 300.0 * a0
  sum = sum + a0
  
  avg = avg + 400.0 * a2
  sum = sum + a2
  
  avg = avg + 500.0 * a4
  sum = sum + a4
  
  if a0>line_crossing_threashold and a1>line_crossing_threashold and a2>line_crossing_threashold and a3>line_crossing_threashold and a4>line_crossing_threashold and a5>line_crossing_threashold:
    if not currently_crossing:
      line_crossing = line_crossing + 1
      currently_crossing = True
  else:
    currently_crossing = False
    
  threashold_2 = 40.0
  if a0>threashold_2 or a1>threashold_2 or a2>threashold_2 or a3>threashold_2 or a4>threashold_2 or a5>threashold_2:
    line = avg / sum
    factor = 1
    found_line = 1
    lost_line = 0
  else:
    factor = 0
    found_line = 0
    lost_line = lost_line+1
    
    
##############################################################################################################    
    
last_proportional = 0.0
slowdown_factor = 0.0
    
def linefollow_update():
  global lm,rm
  global speed_cmd
  global last_proportional, slowdown_factor, power_difference
  global integral
  global line_crossing
    
  center = 250.0
  
  proportional = float(center - line)
  
  #deadband
  if abs(proportional) < 3:
    proportional = 0
  
  derivative = float(proportional - last_proportional)
  
  last_proportional = proportional
  
  integral = float(integral + proportional)
  if abs(proportional) < 20:
    integral = integral * 0.97
  #integral=0.0
  
  power_difference = proportional*kp + integral*ki + derivative*kd

  #slowdown_factor = (slowdown_factor*2.0 + (1.0 - ( abs(proportional) / 2200.0 ))) / 3.0
  #speed = speed_cmd * slowdown_factor
  
  if abs(proportional) > slowdown_threashold:
    slowdown_factor = slowdown_factor + 2
  else:
    if slowdown_factor > 0: 
      slowdown_factor = slowdown_factor - 1

  if slowdown_factor > max_slowdown:
    slowdown_factor = max_slowdown


  if speed_cmd < target_speed:
    speed_cmd = speed_cmd + 1

  if speed_cmd > target_speed:
    speed_cmd = speed_cmd - 2

    
  speed = speed_cmd - slowdown_factor
  
  
  if power_difference > speed:
      power_difference = speed;
  if power_difference < -speed:
      power_difference = -speed;
   
  if power_difference < 0:
      #set_motors(speed+power_difference, speed);
      rm = (rm*1 + speed+power_difference) / 2
      lm = (lm*1 + speed) / 2
  else:
      #set_motors(speed, speed-power_difference); 
      rm = (rm*1 + speed) / 2
      lm = (lm*1 + speed-power_difference ) / 2 
  
  
  if lm<0:
    lm=0
    
  if rm<0:
    rm=0
        



def linefollow_info():

  s = "%4d,%4d,%4d,%4d,%4d,%4d   line=%5.2f  adjust=%4d  int=%7d  pd=%4d  sf=%5.2f  lm,rm=%3d,%3d lc=%2d" % ( a3, a1, a5,  a0, a2, a4 ,line , adjust , integral, power_difference, slowdown_factor, lm , rm , line_crossing)
  return s
  
  
def csv():
  s =  "%4d, %4d, %4d, %4d, %4d, %4d, %5.2f, %3d, %3d, %2d" % ( a3, a1, a5,  a0, a2, a4 ,line , lm , rm , line_crossing)
  return s
  
  

if __name__ == '__main__':


  print("installing SIGINT handler")
  signal.signal(signal.SIGINT, signal_handler)

  while not done:
    time.sleep(0.008)

    ts1 = time.time()
    
    analog=a_star.read_analog()
    qtr_update(analog)
    linefollow_update()
    a_star.motors(int(lm), int(rm));       
    battery_millivolts = a_star.read_battery_millivolts()

    ts2 = time.time()
  
    t = time.monotonic()
    if t-time_since_last_print > 0.5:
      li = linefollow_info()
      print("t=%7.3f  dt=%6.4f  %s  %4d mV errors=%d" % (t,ts2-ts1, li, battery_millivolts[0], a_star.errors) )
      time_since_last_print = t  

  a_star.motors(0, 0);
  print("exiting....")


