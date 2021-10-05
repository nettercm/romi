#!/usr/bin/env python3
from scanf import scanf
import re
import math
import time
import serial
import signal


f = serial.Serial('/dev/ttyACM1',1000000) #baud rate value does not matter actually
f.flushInput()
f.flushInput()
f.timeout = 0.005

while True:
        b = f.read(5000)
        if len(b) > 0:
            if b[len(b)-1] != 10:  #if the read returned before we got the \r\n, then just do another read
                b = b + f.read(5000)
            l = b.decode("utf-8")
            #print(b)
            r = scanf('%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d',l)
            #print(l)
            print(r)
 
while True:
    #l = f.readline() #.decode("utf-8")
    waiting = f.in_waiting
    if waiting > 0:
        b = f.read(waiting)
        print(b)
    else:
        time.sleep(0.005)



