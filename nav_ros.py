#!/usr/bin/env python3

import time

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import cos, sin, pi, atan2
from std_msgs.msg import Int16, Int32, Float32MultiArray
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import tf
import rospy
import os
import fcntl
import termios
import math
import threading
import sys
import signal
import numpy as np

import nav
from scanf import scanf

#for automatic restart if script changes
from os.path import getmtime
file_time = getmtime(__file__)


import reconfiguration as r

nav.downramp = 0.3
r.params.add("downramp", r.double_t, 0, "distance from target at which we start to ramp speed down",    0.3, 0.0,   1.0)

nav.angular_speed = 1.9
r.params.add("angular_speed", r.double_t, 0, "max navigation turn rate",    1.9,  0.0,   6.28)

nav.linear_speed = 0.3
r.params.add("linear_speed", r.double_t, 0, "max navigation linear speed",    0.3,  0.0,   1.0)

nav.error_circle = 0.013
r.params.add("error_circle", r.double_t, 0, "error circle - how close do we need to be to the target?",    0.013,  0.01,   0.2)

nav.bearing_threashold = 50.0    
r.params.add("bearing_threashold", r.double_t, 0, "if bearing is off by more than that, we steer hard and slow way way down",    50.0,  0.0,   120.0)

nav.slow_down_factor   = 0.1
r.params.add("slow_down_factor", r.double_t, 0, "if bearing is way off, we slow down by this factor",    0.1,  0.0,   1.0)



def config_callback(config, level):

    nav.downramp            = config['downramp']
    nav.angular_speed       = config['angular_speed']
    nav.linear_speed        = config['linear_speed']
    nav.error_circle        = config['error_circle']
    nav.bearing_threashold  = config['bearing_threashold']
    nav.slow_down_factor    = config['slow_down_factor']

    return config # not sure why this is done - that's what the example did.....



'''
ros:  
theta = 0, going forward =>  x increases
turning right => theta decreases
turning left  => theta increases

'''
# these globals will get updated every time new X/Y/Theta odometry data comes in via the subscribed /odom ROS topic
X_position = 0.0
Y_position = 0.0
Theta = 0.0
odometry_ready = False
ranges = [9.0,9.0,9.0,9.0,9.0,9.0,9.0,9.0,9.0,9.0,9.0,9.0]

done = False

def signal_handler(sig, frame):
    global done
    print('You pressed Ctrl+C!')
    done = True



print("installing SIGINT handler")
signal.signal(signal.SIGINT, signal_handler)




def yaw_from_quaternion(orientation : Quaternion):
    w = orientation.w
    x = orientation.x
    y = orientation.y
    z = orientation.z
    theta = atan2((2.0 * (w*z + x*y)), (1.0 - 2.0 * (y*y + z*z)))
    return theta



def odom_callback(msg: Odometry):
    global odometry_ready, X_position, Y_position, Theta
    X_position = msg.pose.pose.position.x
    Y_position = msg.pose.pose.position.y
    Theta = yaw_from_quaternion(msg.pose.pose.orientation)
    odometry_ready = True
    return




def laser_callback(msg : Float32MultiArray):
    global ranges
    ranges = msg.data
    return




myargv = rospy.myargv(argv=sys.argv)
print(sys.argv)
print(myargv)

# process command line parameters
#
if len(myargv) == 1:
    args = "0.0 0.0"
else:
    args = myargv[1]+" "+myargv[2]
scanf_result = scanf('%f %f',args)
print(scanf_result)
X_target = scanf_result[0]
Y_target = scanf_result[1]



# initialize all that ROS stuff....
#
rospy.init_node('navigate',disable_signals=True)
odom_sub = rospy.Subscriber("odom", Odometry,        odom_callback, tcp_nodelay=True)
laser_sub =      rospy.Subscriber("laser_array",     Float32MultiArray, laser_callback,     tcp_nodelay=True)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5, tcp_nodelay=True)
cmd_vel_data = Twist()
rate = rospy.Rate(20)  # loop rate of 20Hz seems sufficient


#wait for odometry input to start arriving....otherwise we don't know our current position
#
t = time.monotonic()
while (not odometry_ready) and (not done):
    rospy.sleep(0.02)
    if time.monotonic() > t + 1.0:
        break
print("%11.3f    Odometry is ready...."% (time.monotonic()))

r.start(config_callback)

while (not rospy.is_shutdown()) and (not done):

    current_time = rospy.Time.now() # ROS time...
    t = current_time.to_sec()         

    # produce the navigation command and determine if we already reached the target
    #
    (acquired, commanded_linear_speed, commanded_angular_speed, distance, bearing) = nav.navigate_target(  X_position, Y_position, Theta,     X_target, Y_target,   ranges  )

    # publish the commanded speed via a standard ROS /cmd_vel mesage of type Twist
    #
    cmd_vel_data.angular.x = 0.0
    cmd_vel_data.angular.y = 0.0
    cmd_vel_data.angular.z = commanded_angular_speed 
    cmd_vel_data.linear.x =  commanded_linear_speed
    cmd_vel_data.linear.y =  0.0
    cmd_vel_data.linear.z =  0.0
    cmd_vel_pub.publish(cmd_vel_data)

    print("%11.3f,   %5.2f,%5.2f,%5.2f,    %5.3f,   %5.2f,%5.2f,%5.2f" % (  t, X_position, Y_position, Theta,     distance,    bearing, commanded_linear_speed, commanded_angular_speed  ))

    # reached?  then we are done
    #
    if acquired:
        #break
        print("we reached")

    if getmtime(__file__) != file_time:
        print("restarting....");    
        cmd_vel_data.angular.x = 0.0;   cmd_vel_data.angular.y = 0.0;   cmd_vel_data.angular.z = 0.0
        cmd_vel_data.linear.x =  0.0;   cmd_vel_data.linear.y =  0.0;   cmd_vel_data.linear.z =  0.0
        cmd_vel_pub.publish(cmd_vel_data);      rospy.sleep(0.5)
        rospy.signal_shutdown('restarting');    rospy.sleep(0.5);       os.execv(__file__, sys.argv)

    # basically a sleep(), but done in such a way that we maintain the desired loop rate,
    # regardless of how long the body of the loop took to execute
    rate.sleep()


cmd_vel_data.angular.x = 0.0;   cmd_vel_data.angular.y = 0.0;   cmd_vel_data.angular.z = 0.0
cmd_vel_data.linear.x =  0.0;   cmd_vel_data.linear.y =  0.0;   cmd_vel_data.linear.z =  0.0
cmd_vel_pub.publish(cmd_vel_data)
print("Exiting...")
rospy.sleep(0.2)
rospy.signal_shutdown("Ctrl-C")
rospy.sleep(0.2)

