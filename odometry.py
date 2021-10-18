
from a_star import AStar
import time
import signal
import sys
import threading
import math
from math import cos, sin, pi
import termios
import fcntl
import os

try:
    from pynput import keyboard
except:
    print("unable to import pynput")


a_star = None

PI = 3.1415926535897932384626433832795
K_rad_to_deg = (180.0/3.1415926535897932384626433832795)
K_deg_to_rad = (PI/180.0)

odo_cml = 0.15271631  # calculated based on wheel, gear and encoder specs...
odo_cmr = 0.15271631  # 0.15171631;
odo_b = 148.0  # 143.0; #between 150 and 136

x1 = 0.0
y1 = 0.0
th1 = 0.0
g_U = 0.0
g_l = 0
g_r = 0
g_dl = 0
g_dr = 0

left_total = 0
right_total = 0
l_target = 0
r_target = 0
l_cmd = 0
r_cmd = 0

speed_increment = 2

done = False


# Parameters
wheeltrack = 0.1400 #0.1400  # 0.1415
wheelradius_L = 0.0348 #0.0364
wheelradius_R = 0.0348
TPR = 1440  # 120 *12
left_ticks = 0
right_ticks = 0
last_left_ticks = 0
last_right_ticks = 0

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0

initialized1 = 0
initialized2 = 0

#######################################################################


def on_press_callback(key):
    global l_cmd, r_cmd, l_target, r_target, speed_increment
    print('You pressed a key')
    if key == keyboard.Key.up:
        l_target += speed_increment
        r_target += speed_increment
    if key == keyboard.Key.down:
        l_target -= speed_increment
        r_target -= speed_increment
    if key == keyboard.Key.left:
        l_target -= speed_increment
        r_target += speed_increment
    if key == keyboard.Key.right:
        l_target += speed_increment
        r_target -= speed_increment
    if key == keyboard.Key.space:
        l_target = 0
        r_target = 0
        l_cmd = 0
        r_cmd = 0
    return True

#######################################################################


def on_release_callback(key):
    print('You released a key')
    if key == keyboard.Key.esc:
        print('You pressed Esc')
        # return False
    return True

#######################################################################


def signal_handler(sig, frame):
    global done
    print('You pressed Ctrl+C!')
    a_star.motors(0, 0)
    done = True
    # sys.exit(0)

#######################################################################


def debug_thread(name):
    global g_l, g_r, g_dl, g_dr
    while 1:
        print(g_x, g_y, K_rad_to_deg*g_theta)
        time.sleep(0.2)

#######################################################################


def odometry_update_v1(delta_L, delta_R):
    #  float d_theta, d_x, d_y, l, r;
    #  float d_Ul, d_Ur, d_U;
    global x1, y1, th1, g_U, g_l, g_r, g_dl, g_dr
    global initialized1

    if not initialized1:
        initialized1 = 1
        return

    g_l += delta_L
    g_r += delta_R
    g_dl = delta_L
    g_dr = delta_R

    l = float(delta_L)
    r = float(delta_R)

    d_Ul = odo_cml * l
    d_Ur = odo_cmr * r
    d_U = (d_Ul + d_Ur) / 2.0
    d_theta = (d_Ur - d_Ul) / odo_b

    # update our absolute position
    d_x = d_U * cos(th1)
    d_y = d_U * sin(th1)
    x1 = x1 + d_x
    y1 = y1 + d_y
    th1 = th1 + d_theta
    g_U = g_U + d_U

    if th1 > 1.0*PI:
        th1 -= 2.0*PI
    # if(g_theta < -2.0*PI) g_theta += 2.0*PI;
    if th1 < -1.0*PI:
        th1 += 2.0*PI


#######################################################################


def speed_control_thread(name):
    global l_cmd, r_cmd, l_target, r_target, left_total, right_total
    left1, right1 = a_star.read_encoders()
    while 1:
        time.sleep(0.02)

        # first let's read...
        left2, right2 = a_star.read_encoders()
        #analog = a_star.read_analog();

        left_delta = left2-left1
        if left_delta > 32000:
            left_delta = 65536 - left_delta
        if left_delta < -32000:
            left_delta = 65536 + left_delta
        right_delta = right2-right1
        if right_delta > 32000:
            right_delta = 65536 - right_delta
        if right_delta < -32000:
            right_delta = 65536 + right_delta
        left_total += left_delta
        right_total += right_delta
        left1 = left2
        right1 = right2
        odometry_update(left_delta, right_delta)

        if left_delta > l_target:
            l_cmd -= 1
        if left_delta < l_target:
            l_cmd += 1
        if right_delta > r_target:
            r_cmd -= 1
        if right_delta < r_target:
            r_cmd += 1

        # now let's write
        a_star.motors(l_cmd, r_cmd)


#######################################################################


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
    normalized = theta % TWO_PI
    normalized = (normalized + TWO_PI) % TWO_PI
    if normalized > pi:
        normalized = normalized - TWO_PI
    return normalized


def odometry_update_v2(delta_L, delta_R, delta_T):

    global initialized2
    global x, y, th
    global vx, vy, vth

    # make sure that we always start at 0,0 when this node is restarted
    # at least during testing, that makes sense
    if not initialized2:
        initialized2 = 1
        return

    dl = 2 * pi * wheelradius_L * delta_L / TPR
    dr = 2 * pi * wheelradius_R * delta_R / TPR
    dc = (dl + dr) / 2
    dt = delta_T  # (current_time - last_time).to_sec()
    dth = (dr-dl)/wheeltrack

    # if dr==dl:
    if abs(dr-dl) < 0.001:
        dx = dr*cos(th)
        dy = dr*sin(th)

    else:
        radius = dc/dth
        iccX = x-radius*sin(th)
        iccY = y+radius*cos(th)
        dx = cos(dth) * (x-iccX) - sin(dth) * (y-iccY) + iccX - x
        dy = sin(dth) * (x-iccX) + cos(dth) * (y-iccY) + iccY - y

    x += dx
    y += dy
    th = (th+dth) % (2 * pi)

    if dt > 0:
        vx = dc/dt  # dx/dt
        vy = 0.0    # dy/dt
        vth = dth/dt
    else:
        vx = 0
        vy = 0
        vth = 0


"""
print("starting debug_thread")
t1=threading.Thread(target=debug_thread, args=(1,));
t1.daemon=True;
t1.start();

print("starting speed_control_thread")
t2=threading.Thread(target=speed_control_thread, args=(1,));
t2.daemon=True;
t2.start();


"""


def odometry_update(left_ticks, right_ticks):
    global last_left_ticks, last_right_ticks

    delta_L = left_ticks - last_left_ticks
    delta_R = right_ticks - last_right_ticks
    last_left_ticks = left_ticks
    last_right_ticks = right_ticks

    if delta_L > 32000:
        delta_L = 65536 - delta_L
    if delta_L < -32000:
        delta_L = 65536 + delta_L
    if delta_R > 32000:
        delta_R = 65536 - delta_R
    if delta_R < -32000:
        delta_R = 65536 + delta_R

    odometry_update_v2(delta_L, delta_R)

    return delta_L, delta_R


def test1():
    global done
    l_cmd = 0
    r_cmd = 0
    counter = 0
    battery = 7400
    while not done:
        time.sleep(0.01)

        left_ticks, right_ticks = a_star.read_encoders()
        delta_L, delta_R = odometry_update(left_ticks, right_ticks)

        if delta_L > l_target:
            l_cmd -= 1
        if delta_L < l_target:
            l_cmd += 1
        if delta_R > r_target:
            r_cmd -= 1
        if delta_R < r_target:
            r_cmd += 1

        a_star.motors(l_cmd, r_cmd)
        # print(l_cmd,r_cmd)

        counter = counter+1
        if counter > 9:
            counter = 0
            time.sleep(0.0005)
            b = a_star.read_battery_millivolts()[0]
            if b > 0:
                print("b=%5d, l_cmd=%4d, r_cmd=%4d, delta_L=%4d, delta_R=%4d, l_target=%4d, r_target=%4d x=%7.5f y=%7.5f th=%8.3f   x1=%7.1f y1=%7.1f th1=%8.3f" % (
                    int(b), l_cmd, r_cmd, delta_L, delta_R, l_target, r_target, x, y, degs(norm(th)), x1, y1, degs(norm(th1))))


if __name__ == '__main__':

    print("installing SIGINT handler")
    signal.signal(signal.SIGINT, signal_handler)

    print("starting keyboard listener")
    listener = keyboard.Listener(
        on_press=on_press_callback, on_release=on_release_callback)
    listener.start()

    a_star = AStar()

    counter = 0

    battery = int(a_star.read_battery_millivolts()[0])

    test1()

    a_star.motors(0, 0)
