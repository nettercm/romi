import time
import signal
import sys
import threading
import math
import termios
import fcntl
import os
import keyboard
from scanf import scanf

import gyro as g
from a_star import AStar
import qtr
import odometry as o

# gyro read+update @ 416Hz == 9% cpu of one core,   i.e. ~2.5% of total available CPU time
# line-follow @ 100Hz = 5% of one core  (includes 1% worth of CPU time for 100Hz print)
# line-follow code takes about 2ms every iteration; this includes analog(), battery(), set_motors() i2c transactions


print(sys.argv)

scanf_result = scanf('%d',sys.argv[1])

if scanf_result != None:
  max_crossings = scanf_result[0]
else:
  max_crossings = 3

print("max_crossings = %d" % (max_crossings) ) 

a_star = AStar()

done = False


analog = None
encoders = None



def signal_handler(sig, frame):
    global done
    print('test.py: You pressed Ctrl+C!')
    #a_star.motors(0, 0);
    done = True
    #sys.exit(0)



def romi_read_data():
  global analog,battery,encoders
  # the following 3 take about 2.2ms
  analog = a_star.read_analog()
  #battery = a_star.read_battery_millivolts()
  encoders = a_star.read_encoders()



f = None
t_last_log = 0
start_logging = False
log_counter = 0

def log_data():
  global f,t_last_log, log_counter
  log_counter = log_counter + 1
  t=time.monotonic()
  if start_logging:
    s= "%6d, %10.5f, %6.5f,   g:, %9.3f, %8.4f, %8.3f, %3d,   e:, %6d, %6d,   o:, %7.5f, %7.5f, %5.2f,   l:,%s\n" % (log_counter, t, t-t_last_log ,g.raw[2] ,g.z_offset, g.angle,g.timing_errors, encoders[0], encoders[1], o.x,o.y,o.degs(o.norm(o.th)), qtr.csv() )
    f.write(s)
  t_last_log=t
  


   
      
if __name__ == '__main__':

  t_print = 0
  t_last_print = 0
  t_romi_update = 0.0
  num_gyro_updates = 0
  num_romi_updates = 0
  last_gyro_update_counter = 0
  last_romi_update_counter = 0
  romi_update_counter = 0
  last_crossing = 0
  
  print("installing SIGINT handler")
  signal.signal(signal.SIGINT, signal_handler)

  
  g.initialize()
  g.run_calibration_loop()
  
  
  #a_star.motors(-40,-45)
  #time.sleep(1.5)
  #a_star.motors(0,0) 
  #time.sleep(1)
  
  blocking = True # non-blocking seems to make it harder to guarantee timing
  
  f = open('data.csv','w')
  
  #let's run through everyting once before entering the loop, so that everything is initialized
  g.update_heading()
  romi_read_data()
  qtr.qtr_update(analog)
  o.odometry_update(encoders[0],encoders[1])  
  """
  a_star.motors(-40, 45); 


  while not done:
    g.update_heading();  log_data()
    t1 = time.monotonic() 
    if t1 - t_romi_update > 0.007:
      romi_read_data()
      t_romi_update = time.monotonic() 
      romi_update_counter = romi_update_counter + 1
      #before we do anything else, let's service the gyro      
      g.update_heading(blocking);  log_data()
      qtr.qtr_update(analog)
      #qtr.linefollow_update()
      li = qtr.linefollow_info()
      o.th = o.rads(g.angle)
      o.odometry_update(encoders[0],encoders[1])  
      g.update_heading(blocking);  log_data()
      
    t_print = time.time()
    
    if done or (t_print - t_last_print >= 0.05):
      g.update_heading(blocking); log_data()  
      num_gyro_updates = g.update_counter - last_gyro_update_counter
      num_romi_updates = romi_update_counter - last_romi_update_counter
      print("angle=%8.3f   timing_errors=%d   li:  %s   x,y,th=%7.5f,%7.5f,%5.2f" % (g.angle,g.timing_errors ,li,o.x,o.y,o.degs(o.norm(o.th))) )
      t_last_print = t_print
      last_gyro_update_counter = g.update_counter
      last_romi_update_counter = romi_update_counter
      
      
      if (g.angle >= 178.0) or (g.angle <= -178.0):
        a_star.motors(0, 0); 
        done = True
        
        
  a_star.motors(0, 0); 
  f.close()
  sys.exit(0)
  """
  while not done:
  
    g.update_heading();  log_data()
    
    t1 = time.monotonic() 
    if t1 - t_romi_update > 0.007:
      romi_read_data()
      t_romi_update = time.monotonic() 
      romi_update_counter = romi_update_counter + 1
      #before we do anything else, let's service the gyro      
      g.update_heading(blocking);  log_data()
      qtr.qtr_update(analog)
      qtr.linefollow_update()
      li = qtr.linefollow_info()
      g.update_heading(); log_data()   

      o.th = o.rads(g.angle)
      o.odometry_update(encoders[0],encoders[1])  

      a_star.motors(int(qtr.lm), int(qtr.rm)); 

      #if keyboard.read_key() == 'x': done = True
      
      if qtr.line_crossing == 0:
        qtr.target_speed = 50

      if qtr.line_crossing != last_crossing:
        last_crossing = qtr.line_crossing
        
        if qtr.line_crossing == 1:
          g.angle = 0.0
          o.x = 0.0
          o.y = 0.0
          o.th = 0.0
          
          qtr.target_speed = 180
          qtr.kp = 0.58
          qtr.kd = 2.5
          qtr.ki = 0.0005
          qtr.slowdown_threashold = 80
          qtr.max_slowdown=10
          
          start_logging = True
          
        if qtr.line_crossing % 2 == 0:
          qtr.target_speed = 50
        else:
          qtr.target_speed = 180


        if qtr.line_crossing == max_crossings:
          done = True
          
      #if (qtr.line_crossing >=3 ) and (qtr.lm < 2) and (qtr.rm < 2):
      #  done = True
      
      
    t_print = time.time()
    
    if done or (t_print - t_last_print >= 0.05):
      g.update_heading(blocking); log_data()  
      num_gyro_updates = g.update_counter - last_gyro_update_counter
      num_romi_updates = romi_update_counter - last_romi_update_counter
      print("angle=%8.3f   timing_errors=%d   li:  %s   x,y,th=%7.5f,%7.5f,%5.2f" % (g.angle,g.timing_errors ,li,o.x,o.y,o.degs(o.norm(o.th))) )
      t_last_print = t_print
      last_gyro_update_counter = g.update_counter
      last_romi_update_counter = romi_update_counter
  
  f.close()
  a_star.motors(0, 0);
  print("exiting....")

