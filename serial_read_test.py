#!/usr/bin/env python3
import time
import serial

# baud rate value does not matter actually
f = serial.Serial('/dev/ttyACM0', 1000000)
f.flushInput()
f.flushInput()
f.write(b'\x1b')
f.timeout = 0.005

while True:
    t1 = time.monotonic()
    b = f.read(48)
    if len(b) > 0:
        while b[len(b)-1] != 10:
            b = b + f.read(1)
        t2 = time.monotonic()
        #print(b)
        print("%9.3f %4.1f %4d" % ( t2, 1000*(t2-t1), len(b)))
        f.write(b'\x1b')
