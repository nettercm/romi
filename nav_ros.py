#!/usr/bin/env python3

import time

from jinja2 import TemplateRuntimeError
print("%11.3f" % (time.monotonic()))  # print some time stamps for profiling purposes

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import cos, sin, pi, atan2
from std_msgs.msg import Int16, Int32
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

print("%11.3f" % (time.monotonic()))  # print some time stamps for profiling purposes

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
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5, tcp_nodelay=True)
cmd_vel_data = Twist()
r = rospy.Rate(20)  # loop rate of 20Hz seems sufficient


#wait for odometry input to start arriving....otherwise we don't know our current position
#
t = time.monotonic()
while (not odometry_ready) and (not done):
    rospy.sleep(0.02)
    if time.monotonic() > t + 1.0:
        break
print("%11.3f    Odometry is ready...."% (time.monotonic()))



while (not rospy.is_shutdown()) and (not done):

    current_time = rospy.Time.now() # ROS time...
    t = time.monotonic()            # wall clock time.... 

    # produce the navigation command and determine if we already reached the target
    #
    (acquired, commanded_linear_speed, commanded_angular_speed, distance, bearing) = nav.navigate_target(  X_position, Y_position, Theta,     X_target, Y_target  )

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
        break

    # basically a sleep(), but done in such a way that we maintain the desired loop rate,
    # regardless of how long the body of the loop took to execute
    r.sleep()


cmd_vel_data.angular.x = 0.0
cmd_vel_data.angular.y = 0.0
cmd_vel_data.angular.z = 0.0
cmd_vel_data.linear.x =  0.0
cmd_vel_data.linear.y =  0.0
cmd_vel_data.linear.z =  0.0
cmd_vel_pub.publish(cmd_vel_data)
print("Exiting...")
rospy.sleep(0.2)
rospy.signal_shutdown("Ctrl-C")
rospy.sleep(0.2)

