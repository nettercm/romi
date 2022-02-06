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
config.params.add("turn_speed", config.double_t, 0, "mturn_speed",    1.5, 0.0,   10.0)

def config_callback(config, level):

    avoid.turn_speed            = config['turn_speed']
    
    return config # not sure why this is done - that's what the example did.....

#############################################  Parameter stuff....  #####################################################

#for automatic restart if script changes
from os.path import getmtime
file_time = getmtime(__file__)



done = 0


t_last_print = time.monotonic()

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

    cmd_vel_data = Twist()

    cmd_vel_data.angular.x = 0.0
    cmd_vel_data.angular.y = 0.0

    cmd_vel_data.linear.x =  0.0
    cmd_vel_data.linear.y =  0.0
    cmd_vel_data.linear.z =  0.0

    result =  avoid.avoid_behavior(r)
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


print("installing SIGINT handler")
signal.signal(signal.SIGINT, signal_handler)



rospy.init_node('avoid')

config.start(config_callback)

cmd_vel_pub =  rospy.Publisher("cmd_vel",       Twist,        queue_size=5,     tcp_nodelay=True)

imu_sub =     rospy.Subscriber("laser_array",     Float32MultiArray, laser_callback,     tcp_nodelay=True)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

rate = rospy.Rate(20)

while not rospy.is_shutdown():


    if getmtime(__file__) != file_time:
        print("restarting....");    
        rospy.signal_shutdown('restarting');    rospy.sleep(0.5);       os.execv(__file__, sys.argv)
    
    rate.sleep()

