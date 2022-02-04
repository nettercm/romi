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



#############################################  Parameter stuff....  #####################################################

import reconfiguration as r

#############################################  Parameter stuff....  #####################################################



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


def sign(x):
    if x >= 0: return 1
    return -1



def avoid():
    global r
    #print(ranges)
    #front = min(r[0],r[1],r[2],r[11],r[10])
    front = min(r[0],r[1],r[11])
    left = min(r[2],r[3])
    right = min(r[10],r[9])

    cmd_vel_data = Twist()

    cmd_vel_data.angular.x = 0.0
    cmd_vel_data.angular.y = 0.0

    cmd_vel_data.linear.x =  0.1
    cmd_vel_data.linear.y =  0.0
    cmd_vel_data.linear.z =  0.0

    print("%5.3f  %5.3f  %5.3f      %4.2f %4.2f    %4.2f %4.2f %4.2f    %4.2f %4.2f" % (left,front,right,  r[3],r[2],r[1],r[0],r[11],r[10],r[9]))
    
    if (front < th):  #or (left < th) or (right < th):
        if left <= right:
            print("turn right")
            cmd_vel_data.angular.z = -3.0
        else: #elif right < front:
            print("turn left")
            cmd_vel_data.angular.z = 3.0

        cmd_vel_pub.publish(cmd_vel_data)
    
    return



def laser_callback(msg : Float32MultiArray):
    global r

    r = msg.data
    avoid()

    return


print("installing SIGINT handler")
signal.signal(signal.SIGINT, signal_handler)



rospy.init_node('avoid')


cmd_vel_pub =  rospy.Publisher("cmd_vel",       Twist,        queue_size=5,     tcp_nodelay=True)

imu_sub =     rospy.Subscriber("laser_array",     Float32MultiArray, laser_callback,     tcp_nodelay=True)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

rate = rospy.Rate(10)

th = 0.3

while not rospy.is_shutdown():


    #if front < th:
    
    rate.sleep()

