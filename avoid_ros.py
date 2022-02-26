#!/usr/bin/env python3

import time
import signal
import sys
import threading
import math
import termios
import fcntl
import os
import numpy as np
import statistics

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Pose,Quaternion,Twist,Vector3,PointStamped
from std_msgs.msg import Int16, Float32MultiArray
from sensor_msgs.msg import LaserScan,PointCloud2,PointCloud

from utilities import *

import avoid



#############################################  Parameter stuff....  #####################################################

import reconfiguration as config

avoid.turn_speed = 1.5
config.params.add("turn_speed", config.double_t, 0, "turn_speed",    1.5, 0.0,   10.0)

avoid.front_min = 0.3
config.params.add("front_min", config.double_t, 0, "front_min",    0.3, 0.1,   1.0)

avoid.side_min = 0.3
config.params.add("side_min", config.double_t, 0, "side_min",    0.3, 0.1,   1.0)

avoid.front_speed = 0.1
config.params.add("front_speed", config.double_t, 0, "front_speed",    0.1, -0.2,   0.5)

avoid.side_speed = 0.3
config.params.add("side_speed", config.double_t, 0, "side_speed",    0.3,  0.1,   0.5)

avoid.vx_slew_rate = 0.35
config.params.add("vx_slew_rate", config.double_t, 0, "vx_slew_rate",    0.35,  0.1,   0.7)

avoid.vth_slew_rate = 7.0
config.params.add("vth_slew_rate", config.double_t, 0, "vth_slew_rate",    7.0, 1.0,   10.0)

def config_callback(config, level):
    avoid.turn_speed            = config['turn_speed']
    avoid.front_min             = config['front_min']
    avoid.side_min              = config['side_min']
    avoid.front_speed           = config['front_speed']
    avoid.side_speed            = config['side_speed']
    avoid.vx_slew_rate          = config['vx_slew_rate']
    avoid.vth_slew_rate         = config['vth_slew_rate']
    return config # not sure why this is done - that's what the example did.....

#############################################  Parameter stuff....  #####################################################



#for automatic restart if script changes
from os.path import getmtime
WATCHED_FILES = [__file__,'avoid.py']
WATCHED_FILES_MTIMES = [(f, getmtime(f)) for f in WATCHED_FILES]

current_vth = 0.0
current_vx  = 0.0

done = 0

commanded_linear_velocity = 0.0
commanded_angular_velocity = 0.0

target_angular_velocity = 0.0
target_linear_velocity = 0.0

nearest_front_obstacle = 0.0

avoidance_angular_velocity = 60

ir_front = 0.0
ir_left = 0.0
ir_right = 0.0

us_left = 0.0
us_right = 0.0

r = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

def signal_handler(sig, frame):
    global done
    print('You pressed Ctrl+C!')
    done = 1
    time.sleep(0.1)



def do_avoid(r):
    global current_vx, current_vth
    
    cmd_vel_data = Twist()

    cmd_vel_data.angular.x = 0.0
    cmd_vel_data.angular.y = 0.0

    cmd_vel_data.linear.x =  0.0
    cmd_vel_data.linear.y =  0.0
    cmd_vel_data.linear.z =  0.0

    result =  avoid.avoid_behavior(r, current_vx,  current_vth)
    if result != None:
        cmd_vel_data.angular.z = result[1]
        cmd_vel_data.linear.x = result[0]
        cmd_vel_pub.publish(cmd_vel_data)
    
    return



def laser_callback(msg : Float32MultiArray):
    global r
    r = msg.data
    do_avoid(r)
    return



def odom_callback(msg : Odometry):
    """
    grab current velocities, so avoid logic can use it
    """
    global current_vx, current_vth
    current_vx = msg.twist.twist.linear.x
    current_vth= msg.twist.twist.angular.z
    return


print("installing SIGINT handler")
signal.signal(signal.SIGINT, signal_handler)



rospy.init_node('avoid')

config.start(config_callback)

cmd_vel_pub =  rospy.Publisher("cmd_vel",       Twist,        queue_size=2,     tcp_nodelay=True,    latch=False)

odom_slow_sub= rospy.Subscriber("odom_slow", Odometry,  odom_callback,     tcp_nodelay=True)

laser_sub =      rospy.Subscriber("laser_array",     Float32MultiArray, laser_callback,     tcp_nodelay=True)


current_time = rospy.Time.now()
last_time = rospy.Time.now()

rate = rospy.Rate(10)  # the loop is not doing anythin.....

while not rospy.is_shutdown():

    # nothing to do in the main loop - logic is triggered from the laser callback

    for f, mtime in WATCHED_FILES_MTIMES:
        if getmtime(f) != mtime:
            print("restarting....");    
            rospy.signal_shutdown('restarting');    rospy.sleep(0.5);       os.execv(__file__, sys.argv)
    
    rate.sleep()

