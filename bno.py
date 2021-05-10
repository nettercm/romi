#!/usr/bin/env python3
from scanf import scanf
import re
import math
import time
import serial



sign = lambda x: math.copysign(1, x)
th1 = 0.0
th2 = 0.0
dth_total = 0.0
dth1_total = 0.0
dth_original = 0.0
dth = 0.0
dps = 0.0
dps_max = 0.0
done = False
t_print = 0.0
f = None
angle = 0.0


def between(x_min,x,x_max):
  if (x >= x_min) and (x <= x_max):
    return True
  else:
    return False
    


def th_delta(th_old, th_new):
  if (abs(th_new) > 170.0) and (sign(th_old) != sign(th_new)):
    if sign(th_new) > 0:
      #clockwise
      return -1.0 * ((180.0 - abs(th_old)) + (180.0 - th_new))
    else:
      return +1.0 * ((180.0 - th_old) + (180.0 - abs(th_new)))
  else:
      return th_new - th_old
      
    

def initialize_v1():
    global f, th1, th2
    #f = open('/dev/ttyACM0','r')
    f = serial.Serial('/dev/ttyACM0',1000000)
    #now force a reset
    f.dtr=False
    f.dtr=True
    #time.sleep(2)
    print("### skipping over initial garbage ###")
    done = False
    while not done:
      try:
        l = f.readline().decode("utf-8")
      except:
        #probably a UnicodeDecodeError
        continue
      #print(l)
      r = re.search('Yaw',l)
      if r != None:
        done = True

    l = f.readline().decode("utf-8")
    r = scanf("%d, %f, %f, %f, %d, %d, %d, %d",l)

    #print(l)
    #print(r)

    print("### start of data stream found ###")

    th2 = r[3]
    th1 = th2
   

#blackpill version
def initialize():
    global f, th1, th2
    f = serial.Serial('/dev/ttyACM1',115200)
    f.flushInput()
    time.sleep(0.1)
    f.flushInput()
    time.sleep(0.1)
    if(f.inWaiting()==0):
      f.write(b'\x1b')
    l = f.readline().decode("utf-8")
    r =  scanf('#, %d, %f, %d, %d',l)

    print("### start of data stream found ###")

    th2 = r[1]
    th1 = th2
    angle = 0.0
   

def update_heading():
    global angle , dth_original, dth_total ,dth, dth1_total, th1, th2
    try:
      l = f.readline().decode("utf-8")
      r = scanf('#, %d, %f, %d, %d',l) #scanf("%d, %f, %f, %f, %d, %d, %d, %d",l)
      dth_original = th_delta(th2,r[1])
    except:
      print("something went wrong! ",l,r)
      dth_original = 0.0
      r = [0,th2,0,0]
    dth_total += dth_original
    dth = dth_original * 1.0047 #1.00551 #1.0055 #1.00556 #1.0055556
    dth1_total += dth
    th1 = th1 + dth
    if th1 > 180.0:
        th1 = -(360.0 - th1)
    if th1 < -180.0:
        th1 = 360.0 + th1
    th2 = r[1]

    #th1 is the current heading - after correction

    #angle = th1
    angle = angle + dth
    if angle > 180.0:
        angle = -(360.0 - angle)
    if angle < -180.0:
        angle = 360.0 + angle

def test():
    global angle, th1, dth, dth1_total, dps, dps_max
    t_print = t = time.monotonic()
    while True:
        update_heading()
        t = time.monotonic()
        if t - t_print > 0.045:
            dps = (dps + abs(dth * 100)) / 2.0
            if dps > dps_max: dps_max = dps
            #print("%7.3f, %8.4f, %8.2f, %8.4f, %8.5f, %8.4f, dps=%6.3f,%6.3f" % (t,
            #th1, dth1_total, dth, dth_original, dth_total, dps, dps_max))
            print("%7.3f, %8.4f, %11.1f" % (t, angle, dth1_total))
            t_print = t

  
if __name__ == '__main__':
  initialize()
  test()
  
  
  
  

  
