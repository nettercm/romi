#!/usr/bin/env python3
from scanf import scanf
import re
import math
import time
import serial
import signal


done = False


def signal_handler(sig, frame):
    global done
    print('You pressed Ctrl+C!')
    #a_star.motors(0, 0);
    # sys.exit(0)
    done = True


def sign(x): return math.copysign(1, x)


heading_uncal_old = 0.0
heading_uncal_new = 0.0

heading_delta_uncal_accumulated = 0.0
heading_delta_calib_accumulated = 0.0

heading_delta_uncal = 0.0
heading_delta_calib = 0.0

heading_calib = 0.0

# the absolute heading returned from the BNO085 tends to "drift" by about 1.2 ... 2.0 degrees for every 360 degrees of rotation
# in other words, if the BNO tells us we have turned by 10.00 degrees, we have acutall turned by 10.00*1.004 =~ 10.04 degrees
# it does not "drift" when stationary
calibration_constant = 1.0042  # 1.0034 #1.0047 #1.00551 #1.0055 #1.00556 #1.0055556
noise_floor = 0.01

dps = 0.0
dps_max = 0.0


done = False
t_print = 0.0
f = None


timestamp = 0


line = [0,0,0,0,0,0,0,0]


def between(x_min, x, x_max):
    if (x >= x_min) and (x <= x_max):
        return True
    else:
        return False


def th_delta(th_old, th_new):
    if (abs(th_new) > 170.0) and (sign(th_old) != sign(th_new)):
        if sign(th_new) > 0:
            # clockwise
            return -1.0 * ((180.0 - abs(th_old)) + (180.0 - th_new))
        else:
            return +1.0 * ((180.0 - th_old) + (180.0 - abs(th_new)))
    else:
        return th_new - th_old


def initialize():
    global f, heading_uncal_old
    # baud rate value does not matter actually
    f = serial.Serial('/dev/ttyACM0', 1000000)
    f.flushInput()
    f.flushInput()
    f.write(b'\x1b')
    l = f.readline().decode("utf-8")
    r = scanf('%d,%f,%d,%d,%d,%d,%d,%d,%d,%d', l)

    print("### start of data stream found ###")

    heading_uncal_old = r[1]

    f.timeout = 0.011


def deinitialize():
    global f
    f.close()


def get_reading():
    r = None
    try:
        b = f.read(50)  # total size is actually 52, so let's read at least 50
        if len(b) > 0:
            # if the read returned before we got the \r\n, then just do another read
            while b[len(b)-1] != 10:
                b = b + f.read(1)
            l = b.decode("utf-8")
            #l = f.readline().decode("utf-8")
            r = scanf('%d,%f,%d,%d,%d,%d,%d,%d,%d,%d', l)
            if len(r) != 10:
                r = None
    except:
        print("something went wrong! ")
        r = None
        # f.flushInput()

    return r


def update_heading_v2():
    global timestamp, heading_calib, heading_uncal_new, heading_delta_uncal, heading_delta_uncal_accumulated, heading_delta_calib, heading_delta_calib_accumulated, heading_uncal_old

    if abs(heading_delta_uncal) < noise_floor:
        heading_delta_uncal = 0.0

    heading_delta_uncal_accumulated += heading_delta_uncal

    heading_delta_calib = heading_delta_uncal * calibration_constant

    heading_delta_calib_accumulated += heading_delta_calib

    heading_calib = heading_calib + heading_delta_calib

    if heading_calib > 180.0:
        heading_calib = -(360.0 - heading_calib)
    if heading_calib < -180.0:
        heading_calib = 360.0 + heading_calib

    return True


def update_heading():
    global timestamp, heading_calib, heading_uncal_new, heading_delta_uncal, heading_delta_uncal_accumulated, heading_delta_calib, heading_delta_calib_accumulated, heading_uncal_old
    try:
        b = f.read(50)  # total size is actually 52, so let's read at least 50
        if len(b) > 0:
            # if the read returned before we got the \r\n, then just do another read
            while b[len(b)-1] != 10:
                b = b + f.read(1)
            l = b.decode("utf-8")
        #l = f.readline().decode("utf-8")
        r = scanf('%d,%f,%d,%d,%d,%d,%d,%d,%d,%d', l)
        timestamp = r[0]
        heading_uncal_new = r[1]
        heading_delta_uncal = th_delta(heading_uncal_old, heading_uncal_new)
    except:
        #print("something went wrong! ")
        # f.flushInput()
        heading_delta_uncal = 0.0
        r = [0, heading_uncal_old, 0, 0]
        return False

    # if abs(heading_delta_uncal) < 0.03:
    #    heading_delta_uncal = 0.0

    heading_delta_uncal_accumulated += heading_delta_uncal
    # 1.0034 #1.0047 #1.00551 #1.0055 #1.00556 #1.0055556
    heading_delta_calib = heading_delta_uncal * 1.0042
    heading_delta_calib_accumulated += heading_delta_calib

    heading_uncal_old = heading_uncal_new  # r[1]

    # th1 is the current heading - after correction

    #angle = th1
    heading_calib = heading_calib + heading_delta_calib
    if heading_calib > 180.0:
        heading_calib = -(360.0 - heading_calib)
    if heading_calib < -180.0:
        heading_calib = 360.0 + heading_calib

    return True


def update():

    global timestamp, line, heading_delta_uncal, heading_calib, heading_uncal_new, heading_uncal_old, heading_delta_calib, heading_delta_calib_accumulated, dps, dps_max, done

    data = get_reading()

    if data != None:
        # print(data)
        f.write(b'\x1b')  # tell the qtpy that we are still here....
        timestamp = data[0]

        heading_uncal_new = data[1]
        heading_delta_uncal = th_delta(
            heading_uncal_old, heading_uncal_new)
        heading_uncal_old = heading_uncal_new

        update_heading_v2()

        dps = (4.0*dps + abs(heading_delta_calib * 100)) / 5.0
        if dps > dps_max:
            dps_max = dps

        line = data[2:10]

    if data != None:
        return True
    else:
        return False


def test_v2():

    global timestamp, line, heading_delta_uncal, heading_calib, heading_uncal_new, heading_uncal_old, heading_delta_calib, heading_delta_calib_accumulated, dps, dps_max, done

    t_print = t = t_last = time.monotonic()

    while done == False:

        result = update()

        if result != True:
            continue

        t = time.monotonic()

        # check for issues associated with timely reading of the data
        flag = " "
        if t - t_last > 0.014:
            flag = "timing!"
        #timing errors usually come in pairs, so no need to flag the second one
        #if t - t_last < 0.006:
        #    flag = "timing!"
        t_last = t

        # if False:
        # if True:
        if t - t_print > 0.0:  # print at 5Hz instead of the full 100Hz
            print("%8d, %7.3f, %8.4f, %8.4f, %11.2f, dps=%6.2f %s" % (timestamp, t,
                  heading_calib, heading_uncal_new, heading_delta_calib_accumulated, dps, flag))
            #print(line)
            t_print = t


def test():
    global heading_calib, heading_uncal_new, heading_delta_calib, heading_delta_calib_accumulated, dps, dps_max, done
    dth1_total_previous = 0.0
    t_print = t = time.monotonic()
    while done == False:
        if update_heading():
            t = time.monotonic()
            if True:  # t - t_print > 0.049:
                f.write(b'\x1b')  # tell the qtpy that we are still here....
                dps = (dps + abs(heading_delta_calib * 100)) / 2.0
                if dps > dps_max:
                    dps_max = dps
                # print("%7.3f, %8.4f, %8.2f, %8.4f, %8.5f, %8.4f, dps=%6.3f,%6.3f" % (t,
                # th1, dth1_total, dth, heading_delta_uncal, dth_total, dps, dps_max))
                dps = 100*(heading_delta_calib_accumulated -
                           dth1_total_previous)
                flag = " "
                if t - t_print > 0.014:
                    flag = "timing!"
                if t - t_print < 0.006:
                    flag = "duplicate?"
                print("%8d, %7.3f, %8.4f, %8.4f, %11.2f, dps=%6.2f %s" % (timestamp, t,
                      heading_calib, heading_uncal_new, heading_delta_calib_accumulated, dps, flag))
                dth1_total_previous = heading_delta_calib_accumulated
                t_print = t


if __name__ == '__main__':
    print(__file__, __name__)
    done = False
    print("installing SIGINT handler")
    signal.signal(signal.SIGINT, signal_handler)
    initialize()
    test_v2()
    deinitialize()

else:
    print(__file__, __name__)
