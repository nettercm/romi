#!/usr/bin/env python3

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

from a_star import AStar
import odometry as o



def signal_handler(sig, frame):
    global done
    print('test.py: You pressed Ctrl+C!')
    #a_star.motors(0, 0);
    done = True
    #sys.exit(0)



a_star = AStar()

lm = 0
rm = 0
update_interval = 0.1
duration = 0
done = False
t_print = 0
t_last_print = 0
t_romi_update = 0.0
romi_update_counter = 0


def test1():
  global t_romi_update, lm, rm, done, update_interval, duration, t_print

  a_star.motors(lm,rm)
  time.sleep(2.0)
  b = a_star.read_battery_millivolts()[0]
  encoders = a_star.read_encoders()
  o.odometry_update(encoders[0],encoders[1])
  o.x = 0
  o.y = 0
  o.th = 0
  
  dlf = 0.0
  drf = 0.0
  
  while not done:
  
    b = (b*63.0 + float(a_star.read_battery_millivolts()[0]))/64.0
  
    if time.monotonic() - t_romi_update > update_interval:
      t_romi_update = time.monotonic()
      e = a_star.read_encoders()
      a_star.motors(int(lm),int(rm))
      dl,dr = o.odometry_update(e[0],e[1])
      dlf = (1.0*dlf + float(dl)) / 2.0
      drf = (1.0*drf + float(dr)) / 2.0
      print("%9.3f, %4d, %4d, %5.1f, %5.1f, %6d, %6d, %7.1f" % (t_romi_update, lm, rm, dlf*(1.0/update_interval), drf*(1.0/update_interval), e[0], e[1], b))


def test2():
  global t_romi_update, lm, rm, done, update_interval, duration, t_print

  i = 0 
  j = 0
  b = [0,0,0,0,0,0,0,0]
  lm_cmd_array = [0,0,0,0,0,0,0,0]
  rm_cmd_array = [0,0,0,0,0,0,0,0]
  dl_array = [0,0,0,0,0,0,0,0]
  dr_array = [0,0,0,0,0,0,0,0]
  dlf = 0.0
  drf = 0.0
  encoders = a_star.read_encoders()
  o.odometry_update(encoders[0],encoders[1])
  o.odometry_update(encoders[0],encoders[1])
  o.x = 0
  o.y = 0
  o.th = 0
  rm_cmd = 0
  lm_cmd = 0

  next_update_time = time.monotonic() + update_interval

  while not done:
  
    if time.monotonic() >= next_update_time:   #- t_romi_update > update_interval:
      #t_romi_update = time.monotonic()
      next_update_time += update_interval

      bat = float(a_star.read_battery_millivolts()[0])
      if a_star.error:
        bat = float(a_star.read_battery_millivolts()[0])

      b[i] = bat
      bat = (b[0]+b[1]+b[2]+b[3]+b[4]+b[5]+b[6]+b[7])/8
      i += 1
      if i > 7: i = 0

      e = a_star.read_encoders()
      if a_star.error:
        e = a_star.read_encoders()
      dl,dr = o.odometry_update(e[0],e[1])
      if dr < rm: rm_cmd +=2
      if dr > rm: rm_cmd -=2
      if dl < lm: lm_cmd +=2
      if dl > lm: lm_cmd -=2
      if lm_cmd > 400:  lm_cmd = 400
      if lm_cmd < -400: lm_cmd = -400
      if rm_cmd > 400:  rm_cmd = 400
      if rm_cmd < -400: rm_cmd = -400
      a_star.motors(lm_cmd,rm_cmd)
      lm_cmd_array[j] = lm_cmd
      rm_cmd_array[j] = rm_cmd
      dl_array[j] = dl
      dr_array[j] = dr
      j += 1
      if j > 7: j = 0
      if True: #time.monotonic() - t_print > 0.05:
        t_print = time.monotonic()
        lm_cmd_avg = (lm_cmd_array[0]+lm_cmd_array[1]+lm_cmd_array[2]+lm_cmd_array[3]+lm_cmd_array[4]+lm_cmd_array[5]+lm_cmd_array[6]+lm_cmd_array[7])/8.0
        rm_cmd_avg = (rm_cmd_array[0]+rm_cmd_array[1]+rm_cmd_array[2]+rm_cmd_array[3]+rm_cmd_array[4]+rm_cmd_array[5]+rm_cmd_array[6]+rm_cmd_array[7])/8.0
        dl_avg = (dl_array[0]+dl_array[1]+dl_array[2]+dl_array[3]+dl_array[4]+dl_array[5]+dl_array[6]+dl_array[7])/8.0
        dr_avg = (dr_array[0]+dr_array[1]+dr_array[2]+dr_array[3]+dr_array[4]+dr_array[5]+dr_array[6]+dr_array[7])/8.0
        print("%9.3f, %4d, %4d, %4d, %4d, %5.1f, %5.1f, %4d, %4d, %5.1f, %5.1f, %6d, %6d, %7.1f, %2d"  % (t_print, lm, rm, lm_cmd, rm_cmd, lm_cmd_avg, rm_cmd_avg, dl, dr, dl_avg, dr_avg, e[0], e[1], bat, a_star.errors))

    sleep_time = (next_update_time - time.monotonic()) - 0.0005
    if sleep_time < 0: sleep_time = 0
    time.sleep(sleep_time)

  done = False

  while not done:
    if time.monotonic() - t_romi_update > update_interval:
      t_romi_update = time.monotonic()
      e = a_star.read_encoders()
      if a_star.error:
        e = a_star.read_encoders()
      dl,dr = o.odometry_update(e[0],e[1])
      if dr < 0: rm_cmd +=5
      if dr > 0: rm_cmd -=5
      if dl < 0: lm_cmd +=5
      if dl > 0: lm_cmd -=5
      if abs(dr) < 5: rm_cmd = 0
      if abs(dl) < 5: lm_cmd = 0
      a_star.motors(lm_cmd,rm_cmd)
      if (lm_cmd==0) and (rm_cmd==0): done = True
      if True: #time.monotonic() - t_print > 0.05:
        t_print = time.monotonic()
        print("%9.3f, %4d, %4d, %4d, %4d, %5.1f, %5.1f, %4d, %4d, %5.1f, %5.1f, %6d, %6d, %7.1f, %2d"  % (t_print, lm, rm, lm_cmd, rm_cmd, lm_cmd_avg, rm_cmd_avg, dl, dr, dl_avg, dr_avg, e[0], e[1], bat, a_star.errors))


if __name__ == '__main__':

  
  print("installing SIGINT handler")
  signal.signal(signal.SIGINT, signal_handler)

  print(sys.argv)

  if len(sys.argv) == 1:
    args = "10 -10 100 100"
  else:
    args = sys.argv[1]+" "+sys.argv[2]+" "+sys.argv[3]+" "+sys.argv[4]

  print(args)
  
  scanf_result = scanf('%d %d %f %d',args)
  
  if scanf_result != None:
    lm = scanf_result[0]
    rm = scanf_result[1]
    update_interval = scanf_result[2]
    duration = scanf_result[3]
  else:
    lm = 50
    rm = -50
    update_interval = 0.1
    duration = 1
  
  print("lm=%d   rm=%d   ui=%f  d=%d" % (lm,rm,update_interval,duration) ) 
  
  test2()

  a_star.motors(0,0)

