#!/usr/bin/env python3
from scanf import scanf
import re
import math
import time
import serial
import signal


done=False

def signal_handler(sig, frame):
  global done
  #print('You pressed Ctrl+C!')
  #a_star.motors(0, 0);
  #sys.exit(0)
  done=True



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
angle_raw = 0.0
qtpy_timestamp = 0

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
      
    



def initialize():
    global f, th1, th2
    f = serial.Serial('/dev/ttyACM0',1000000) #baud rate value does not matter actually
    f.flushInput()
    f.flushInput()
    f.write(b'\x1b')
    l = f.readline().decode("utf-8")
    r =  scanf('%d,%f,%d,%d,%d,%d,%d,%d,%d,%d',l)

    print("### start of data stream found ###")

    th2 = r[1]
    th1 = th2
    angle = 0.0

    f.timeout = 0.009
   

def update_heading():
    global qtpy_timestamp, angle , angle_raw, dth_original, dth_total ,dth, dth1_total, th1, th2
    try:
        b = f.read(50)  #total size is actually 52, so let's read at least 50
        if len(b) > 0:
            while b[len(b)-1] != 10:  #if the read returned before we got the \r\n, then just do another read
                b = b + f.read(1)
            l = b.decode("utf-8")
        #l = f.readline().decode("utf-8")
        r = scanf('%d,%f,%d,%d,%d,%d,%d,%d,%d,%d',l)
        qtpy_timestamp = r[0]
        angle_raw = r[1]
        dth_original = th_delta(th2,angle_raw)
    except:
      #print("something went wrong! ")
      #f.flushInput()
      dth_original = 0.0
      r = [0,th2,0,0]
      return False
      
    #if abs(dth_original) < 0.03:
    #    dth_original = 0.0
        
    dth_total += dth_original
    dth = dth_original * 1.0042 #1.0034 #1.0047 #1.00551 #1.0055 #1.00556 #1.0055556
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
        
    return True

def test():
    global angle, angle_raw, th1, dth, dth1_total, dps, dps_max, done
    dth1_total_previous = 0.0
    t_print = t = time.monotonic()
    while done==False:
        if update_heading():
            t = time.monotonic()
            if True: #t - t_print > 0.049:
                f.write(b'\x1b') #tell the qtpy that we are still here....
                dps = (dps + abs(dth * 100)) / 2.0
                if dps > dps_max: dps_max = dps
                #print("%7.3f, %8.4f, %8.2f, %8.4f, %8.5f, %8.4f, dps=%6.3f,%6.3f" % (t,
                #th1, dth1_total, dth, dth_original, dth_total, dps, dps_max))
                dps = 100*(dth1_total - dth1_total_previous)
                flag = " "
                if t - t_print > 0.014:
                    flag = "timing!"
                if t - t_print < 0.006:
                    flag = "duplicate?"
                print("%8d, %7.3f, %8.4f, %8.4f, %11.2f, dps=%6.2f %s" % (qtpy_timestamp, t, angle, angle_raw, dth1_total, dps, flag))
                dth1_total_previous = dth1_total
                t_print = t

  
if __name__ == '__main__':
  done=False
  print("installing SIGINT handler")
  signal.signal(signal.SIGINT, signal_handler)
  initialize()
  test()
  
  
  
  

  
