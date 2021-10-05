#!/usr/bin/env python3

from pyrplidar import PyRPlidar
import time

lidar = PyRPlidar()

def simple_scan():

    count = 0
    last_count = 0
    total_scans = 0

    lidar.connect(port="/dev/ttyS0", baudrate=115200, timeout=3)
    #lidar.reset()
    # Linux   : "/dev/ttyUSB0"
    # MacOS   : "/dev/cu.SLAB_USBtoUART"
    # Windows : "COM5"

                  
    lidar.stop()
    lidar.set_motor_pwm(500)
    #time.sleep(2)
    
    try:
        scan_generator = lidar.start_scan_express(2)
    except:
        lidar.reset()
        lidar.diconnect()
        return
    
    valid_scans = 0 
    
    for count, scan in enumerate(scan_generator()):
        #print(count, scan)
        #if count == 20: break
        if scan.distance > 0.0:
            valid_scans += 1
        if scan.start_flag:
            total_scans += 1
            samples_per_scan = count-last_count
            print("%11.3f  %4d  %4d start  valid=%d" % (time.monotonic(), count, samples_per_scan, valid_scans))
            last_count = count
            valid_scans=0
            #print(scan)
            #if total_scans > 3 and samples_per_scan < 200:
            #    lidar.stop()
            #    lidar.reset()

    lidar.reset()
    lidar.stop()
    lidar.set_motor_pwm(0)

    
    lidar.disconnect()


if __name__ == "__main__":
    try:
        simple_scan()
    except KeyboardInterrupt:
        print('Stoping.')
        lidar.reset()
        lidar.disconnect()
