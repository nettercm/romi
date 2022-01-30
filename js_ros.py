#!/usr/bin/env python3


# standard imports - probably don't need all of those
import time
import signal
import sys
import threading
import termios
import fcntl
import os
from math import cos, sin, pi

# ros imports
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,  Pose,  Quaternion,  Twist,  Vector3,  PointStamped
from std_msgs.msg import Int16, Int32, Int32MultiArray

# "driver"
import js_linux as js

import reconfiguration as r

done = False
joystick_is_idle = False
t_idle = 0.0


idle_timeout = 3.0           
r.params.add("idle_timeout", r.double_t, 0, "joystick goes idle after this amount of inactivity",    3.0, 0.0,   10.0)

auto_idle = True
r.params.add("auto_idle", r.bool_t, 0, "joystick automatically goes idel after some time if this is True",   True)


def config_callback(config, level):
    global idle_timeout, auto_idle

    idle_timeout = config['idle_timeout']
    auto_idle    = config['auto_idle']
    
    return config # not sure why this is done - that's what the example did.....



# catch Ctrl-C
def signal_handler(sig, frame):
    global done
    print('You pressed Ctrl+C!')
    # sys.exit(0)
    # set the flag....
    done = True

# catch Ctrl-C
signal.signal(signal.SIGINT, signal_handler)


rospy.init_node('joystick')

joy_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5, tcp_nodelay=True)

rate = rospy.Rate(20)

js.js_init()

r.start(config_callback)

while (not done) and (not rospy.is_shutdown()):

    y =  -js.axis_states['y']    # left  joystick, up/down axis
    rx = -js.axis_states['rx']   # right joystick  left/right axis  # use rz for gamesir controller

    # convert the joystick values into linear and angular velocity up to "max_speed"
    js_linear_velocity = y    * 0.7    # set max linear velocity to 0.7 meters per second
    js_angular_velocity = rx  * 6.28   # set max angular velocity to 360 degrees per second

    cmd_vel_data = Twist()

    cmd_vel_data.angular.x = 0.0
    cmd_vel_data.angular.y = 0.0
    cmd_vel_data.angular.z = js_angular_velocity 

    cmd_vel_data.linear.x =  js_linear_velocity
    cmd_vel_data.linear.y =  0.0
    cmd_vel_data.linear.z =  0.0


    if joystick_is_idle == False:
        joy_pub.publish(cmd_vel_data)   # only publish if joystick is not idle

    if ( abs(js_angular_velocity) < 0.01 ) and ( abs(js_linear_velocity) < 0.01 ) and js.button_states['a']==0:
        t_idle = t_idle + 0.05
        if (t_idle > idle_timeout) and (auto_idle):
            if joystick_is_idle == False:
                print("joystick idle")  # print only on state change
            joystick_is_idle = True
    else:
        if joystick_is_idle == True:
            print("joystick is busy")
        t_idle = 0.0
        joystick_is_idle = False

    #if 'b' is pressed (which os the O on the PS3 controller), immediately go idle
    if js.button_states['b']==1:
        if joystick_is_idle == False:
            print("joystick idle")  # print only on state change
        joystick_is_idle = True
        

    rate.sleep()



js.js_deinit()
