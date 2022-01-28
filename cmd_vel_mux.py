#!/usr/bin/env python3

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import rospy
import os
import fcntl
import termios
import math
import threading
import sys
import signal
import time


input_1_active = False # indicates if we have recent activity on input 1
t_input_1 = 0.0        # timestamp associated with last activity on input 1

input_2_active = False
t_input_2 = 0.0

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    #sys.exit(0)
    rospy.signal_shutdown("Ctrl+C")

#######################################################################


print("installing SIGINT handler")
signal.signal(signal.SIGINT, signal_handler)



def cmd_vel_callback(msg : Twist):
    """
    The cmd_vel_callback() gets called every time a new message of type /cmd_vel is received.
    We simply convirt it into target speed for left and right wheel
    """
    global input_1_active, t_input_1, input_2_active, t_input_2

    # extract the information we need from the incoming message and populate the outgoing message structure
    cmd_vel_data = Twist()
    cmd_vel_data.angular.x = msg.angular.x
    cmd_vel_data.angular.y = msg.angular.y
    cmd_vel_data.angular.z = msg.angular.z
    cmd_vel_data.linear.x =  msg.linear.x
    cmd_vel_data.linear.y =  msg.linear.y
    cmd_vel_data.linear.z =  msg.linear.z

    # if input 1 is active, publish it immediately
    if msg._connection_header['topic'] == '/cmd_vel_in_1':  
        #print(msg._connection_header['topic'])
        input_1_active = True
        t_input_1 = time.monotonic()
        cmd_vel_out_pub.publish(cmd_vel_data)         

    # if input 1 is active, only publish if input 1 is not active
    if msg._connection_header['topic'] == '/cmd_vel_in_2':  
        input_2_active = True
        t_input_2 = time.monotonic()
        if not input_1_active:
            #print(msg._connection_header['topic'])
            cmd_vel_out_pub.publish(cmd_vel_data)         


#######################################################################



rospy.init_node('cmd_vel_mux')


cmd_vel_out_pub =  rospy.Publisher("cmd_vel",       Twist,        queue_size=5,  tcp_nodelay=True)

cmd_vel_in_1_sub = rospy.Subscriber("cmd_vel_in_1", Twist,        cmd_vel_callback, tcp_nodelay=True)
cmd_vel_in_2_sub = rospy.Subscriber("cmd_vel_in_2", Twist,        cmd_vel_callback, tcp_nodelay=True)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(20)

while not rospy.is_shutdown():

    current_time = rospy.Time.now()
    t = time.monotonic()

    # after a short amount of inactivity on any given input, mark it as inactive
    if input_1_active:
        if t - t_input_1 > 0.2:
            input_1_active = False

    if input_2_active:
        if t - t_input_2 > 0.2:
            input_2_active = False

    r.sleep()



print("Exiting....")
