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
import ctypes

import gyro as g
from a_star import AStar
import qtr
import odometry as o
import bno 

libcap = ctypes.CDLL('libcap.so.2')

a_star = AStar()
analog = [0,0,0,0,0,0]
encoders = [0,0]


def gyro_thread_function():
    global a_star, analog,battery,encoders
    libcap.prctl(15, 'gyro_thread'.encode())
    analog = a_star.read_analog()
    encoders = a_star.read_encoders()
    o.th = 0.0
    o.odometry_update(encoders[0],encoders[1])  
    g.initialize()
    g.run_calibration_loop()
    while True:
        g.update_heading()
        time.sleep(0.0008)
        g.update_heading()
        time.sleep(0.0008)
        g.update_heading()
        time.sleep(0.0008)
        g.update_heading()
        analog = a_star.read_analog()
        encoders = a_star.read_encoders()
        o.th = o.rads(g.angle)
        o.odometry_update(encoders[0],encoders[1])  



def bno_thread_function():
    libcap.prctl(15, 'bno_thread'.encode())
    bno.initialize()
    while True:
        bno.update_heading()



def romi_thread_function():
    global a_star, analog,battery,encoders
    libcap.prctl(15, 'romi_thread'.encode())
    while True:
        # the following 3 take about 2.2ms
        analog = a_star.read_analog()
        #battery = a_star.read_battery_millivolts()
        encoders = a_star.read_encoders()
        time.sleep(0.008)



bno_thread = threading.Thread(target=bno_thread_function)
bno_thread.start()

time.sleep(2)

gyro_thread = threading.Thread(target=gyro_thread_function)
gyro_thread.start()

#romi_thread = threading.Thread(target=romi_thread_function)
#romi_thread.start()


while True:
    print("gyro: angle=%8.3f  te=%d,%7.4f      bno: angle=%8.3f    encoders=%6d,%6d    x,y,th=%7.5f,%7.5f,%5.2f" % (g.angle, g.timing_errors, g.large_d_t, bno.angle, encoders[0], encoders[1] , o.x, o.y, o.degs(o.norm(o.th))) )
    time.sleep(0.1)