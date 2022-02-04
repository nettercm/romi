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

from utilities import *

done = False
joystick_is_idle = False
t_idle = 0.0

current_vth = 0.0
current_vx  = 0.0


max_linear_velocity = 0.7
r.params.add("max_linear_velocity", r.double_t, 0, "maximum linear velocity that will be published",    0.7, 0.0,   1.0)

max_angular_velocity = 6.28
r.params.add("max_angular_velocity", r.double_t, 0, "maximum angular velocity that will be published",    6.28, 0.0,   12.58)

linear_velocity_slew_rate = 1.0
r.params.add("linear_velocity_slew_rate", r.double_t, 0, "linear_velocity_slew_rate",    1.0, 0.0,   1.0)

angular_velocity_slew_rate = 6.28
r.params.add("angular_velocity_slew_rate", r.double_t, 0, "angular_velocity_slew_rate",    6.28, 0.0,   12.58)

auto_idle = True
r.params.add("auto_idle", r.bool_t, 0, "joystick automatically goes idel after some time if this is True",   True)

idle_timeout = 3.0           
r.params.add("idle_timeout", r.double_t, 0, "joystick goes idle after this amount of inactivity",    3.0, 0.0,   10.0)



def config_callback(config, level):
    global idle_timeout, auto_idle, max_linear_velocity, max_angular_velocity

    idle_timeout            = config['idle_timeout']
    auto_idle               = config['auto_idle']
    max_linear_velocity     = config['max_linear_velocity']
    max_angular_velocity    = config['max_angular_velocity']
    
    return config # not sure why this is done - that's what the example did.....



def odom_callback(msg : Odometry):
    global current_vx, current_vth
    current_vx = msg.twist.twist.linear.x
    current_vth= msg.twist.twist.angular.z
    return

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

odom_sub= rospy.Subscriber("odom_slow",     Odometry, odom_callback,     tcp_nodelay=True)

rate = rospy.Rate(20)

js.js_init()

r.start(config_callback)

while (not done) and (not rospy.is_shutdown()):

    y =  -js.axis_states['y']    # left  joystick, up/down axis
    rx = -js.axis_states['rx']   # right joystick  left/right axis  # use rz for gamesir controller

    # convert the joystick values into linear and angular velocity up to "max_speed"
    js_linear_velocity = y    * max_linear_velocity    # set max linear velocity to 0.7 meters per second
    js_angular_velocity = rx  * max_angular_velocity   # set max angular velocity to 360 degrees per second

    cmd_vel_data = Twist()

    cmd_vel_data.angular.x = 0.0
    cmd_vel_data.angular.y = 0.0
    cmd_vel_data.angular.z = slew(current_vth, js_angular_velocity, angular_velocity_slew_rate) 
 
    cmd_vel_data.linear.x =  slew(current_vx , js_linear_velocity, linear_velocity_slew_rate)
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
