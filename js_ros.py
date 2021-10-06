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


done = False


# catch Ctrl-C
def signal_handler(sig, frame):
    global done
    print('You pressed Ctrl+C!')
    # sys.exit(0)
    # set the flag....
    done = True

# catch Ctrl-C
signal.signal(signal.SIGINT, signal_handler)


rospy.init_node('joystick_publisher')

joy_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5, tcp_nodelay=True)

r = rospy.Rate(20)

js.js_init()

while (not done) and (not rospy.is_shutdown()):

    y = -js.axis_states['y']    # left  joystick, up/down axis
    rx = -js.axis_states['rx']   # right joystick  left/right axis

    # convert the joystick values into linear and angular velocity up to "max_speed"
    js_linear_velocity = y     # * max_speed
    js_angular_velocity = rx   # * max_speed

    cmd_vel_data = Twist()

    cmd_vel_data.angular.x = 0.0
    cmd_vel_data.angular.y = 0.0
    cmd_vel_data.angular.z = js_angular_velocity

    cmd_vel_data.linear.x =  js_linear_velocity
    cmd_vel_data.linear.y =  0.0
    cmd_vel_data.linear.z =  0.0

    joy_pub.publish(cmd_vel_data)

    r.sleep()



js.js_deinit()
