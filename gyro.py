#!/usr/bin/python

import time, math
from time import sleep
from math import pi
from lsm6ds33 import LSM6DS33

imu = None
x_offset = 0.0
y_offset = 0.0
z_offset = 0.0
angle = 0.0
last_angle = 0.0
angle_delta = 0.0
t_now = 0.0
t_last = 0.0
d_t = 0.0  

drift = 0.0 # 0.008
last_turn_rate = 0.0
turn_rate = 0.0  
timing_errors = 0
large_d_t = 0.0

update_counter = 0

raw = None

done = False


def rads(degrees):
    """
    convert from degrees to radians
    """
    return degrees * pi / 180.0



def degs(radians):
    """
    convert from radians to degrees
    """
    return radians * 180 / pi



def norm(theta):
    """
    normalize the angle theta so that it always falls between -pi and +pi
    """
    TWO_PI = 2.0 * pi
    normalized = theta % TWO_PI;
    normalized = (normalized + TWO_PI) % TWO_PI;
    if normalized > pi:
        normalized = normalized - TWO_PI;
    return normalized


#if the data is ready, the following takes ~850us  (getStatus = 120us,   getGyroscopeRaw = 725us
def get_next_reading(blocking=True):
  global imu
  status = imu.getStatus()
  if blocking:
    while status != 7:
      time.sleep(0.0005)
      status = imu.getStatus()
    data = imu.getGyroscopeRaw()
    return data
  elif status == 7:
    data = imu.getGyroscopeRaw()
    return data
  else:
    data = [0,0,0]
    return data    
  

  
def calibrate(iterations):
  t1 = time.time()
  x = 0.0
  y = 0.0
  z = 0.0
  for i in range(0,iterations):
    gyro_data = get_next_reading()
    x = x + gyro_data[0]
    y = y + gyro_data[1]
    z = z + gyro_data[2]
  
  x = x / float(iterations)
  y = y / float(iterations)
  z = z / float(iterations)
  
  t2 = time.time()
  return (x,y,z)
  
  
  
def test_update_reate():
  last_status = 0
  t1 = time.time()
  
  data = get_next_reading()
  i = 0 
  while time.time() - t1 < 1.0:
    data = get_next_reading()
    i = i + 1

  print("update rate seems to be %d Hz" % (i))



def run_calibration_loop():
  global x_offset
  global y_offset
  global z_offset
  calibrate(100) #the first time through seems to produce some outliers
  for i in range(0,1):
    t1 = time.time()
    (x_offset,y_offset,z_offset) = calibrate(412) #
    t2 = time.time()
    print("dt = %f, x_offset,y_offset,z_offset = %f,%f,%f" % ((t2-t1), x_offset,y_offset,z_offset) )
  #return (x_off,y_off,z_off)
  


def find_drift():
  calibrate(100) #the first time through seems to produce some outliers
  for i in range(0,20):
    t1 = time.time()
    (x_offset,y_offset,z_offset) = calibrate(824) #
    t2 = time.time()
    print("dt = %f, x_offset,y_offset,z_offset = %f,%f,%f" % ((t2-t1), x_offset,y_offset,z_offset) )



def find_drift_v2():
  global done
  iterations = 0
  t1 = time.time()
  x = 0.0
  y = 0.0
  z = 0.0
  while not done:
    gyro_data = get_next_reading()
    iterations = iterations + 1
    x = x + gyro_data[0]
    y = y + gyro_data[1]
    z = z + gyro_data[2]
    xo = x / float(iterations)
    yo = y / float(iterations)
    zo = z / float(iterations)
    t2 = time.time()
    if t2-t1 > 1.0:
      print("z offset = %8.4f" % (zo) )
      t1 = t2
  
  

def initialize():
  global imu
  imu = LSM6DS33()
  imu.enableLSM()



def update_heading(blocking=True):
  global imu,turn_rate, last_turn_rate, t_now, t_last, angle, last_angle, angle_delta, x_offset, y_offset, z_offset, timing_errors, d_t, update_counter
  global raw
  global large_d_t

  if not blocking:
    status = imu.getStatus()
    if status != 7:
      return
    else:
      raw = imu.getGyroscopeRaw()
      
  else:
    raw = get_next_reading(blocking)
    
  # regardless of blocking vs. non-blocking, at this poing g contains the gyro data
    
  turn_rate = raw[2]
  t_now = time.time()
  turn_rate = turn_rate - z_offset
  if t_last == 0:
    t_last = t_now
    last_turn_rate = turn_rate
    return
  turn_rate = (turn_rate + 1*last_turn_rate) / 2.0
  last_turn_rate = turn_rate
  d_t = (t_now - t_last) 
  if d_t > 0.0038:
    timing_errors = timing_errors + 1
    large_d_t = d_t
  #the hardware is internally sampling at 800Hz, so in really d_t is fixed
  #d_t = 1.0 / 412.0  # technically 416Hz, but I measured 412H
  t_last = t_now
  #angle_delta = (turn_rate * (0.035+0.000420)) * d_t  #this seems to work quite well
  angle_delta = (turn_rate * (0.035+0.000250)) * d_t
  if abs(turn_rate) < 8.0:
    angle_delta = 0
  angle = angle +  angle_delta #* (7340032.0 / 12.0) / 17578125.0
  angle=degs(norm(rads(angle)))
  update_counter = update_counter + 1
    
 
 
 
def test():
  global turn_rate, last_turn_rate, t_now, t_last, angle, last_angle, angle_delta, x_offset, y_offset, z_offset, timing_errors, d_t, drift

  t_print = 0
  t_last_print = 0
  run_calibration_loop()
  drift = 0.0
  
  while True:
    time.sleep(0.0008)
    update_heading(False)
    t_print = time.time()
    if t_print - t_last_print >= 0.02:
      z_offset = z_offset + drift
      print("angle=%8.3f  delta=%8.4f  d=%7.3f turn_rate=%8.2f  d_t=%8.5f  timing_errors=%d" % (angle,angle-last_angle,angle_delta,turn_rate,d_t,timing_errors) )
      t_last_print = t_print
      last_angle = angle
      update_heading(False)



def determine_overhead():

  t1 = time.monotonic()
  for i in range(0,1000):
    status = imu.getStatus()
  t2 = time.monotonic()
  print("imu.getStatus() takes %8.6f seconds" % ( (t2-t1)/1000.0 ) )

  t1 = time.monotonic()
  for i in range(0,1000):
    data = imu.getGyroscopeRaw()
  t2 = time.monotonic()
  print("imu.getGyroscopeRaw() takes %8.6f seconds" % ( (t2-t1)/1000.0 ) )
  



if __name__ == '__main__':
  initialize()
  determine_overhead()
  test()
  #find_drift()
  #find_drift_v2()
  
    
      
