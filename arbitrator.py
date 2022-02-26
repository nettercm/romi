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


#for automatic restart if script changes
from os.path import getmtime
file_time = getmtime(__file__)
print("%s: last modified: %s" % (__file__, time.ctime(file_time)))

priority = dict()
commands = dict()

priority['/joystick'] = 1
priority['/avoid']    = 2
priority['/navigate'] = 3



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
    global priority,commands

    #cmd_vel_arb_pub.publish(cmd_vel_data) 
    caller = msg._connection_header['callerid']

    if caller in priority:
        commands[caller] = (msg.linear.x , msg.angular.z)

#######################################################################



rospy.init_node('arbitrator')

# the individual behaviors call all publish to /cmd_vel
cmd_vel_sub =      rospy.Subscriber("cmd_vel",          Twist,        cmd_vel_callback, tcp_nodelay=True)

# also allow the individual behaviors to publish on unique topics
cmd_vel_joy_sub =  rospy.Subscriber("cmd_vel_joy",      Twist,        cmd_vel_callback, tcp_nodelay=True)
cmd_vel_nav_sub =  rospy.Subscriber("cmd_vel_nav",      Twist,        cmd_vel_callback, tcp_nodelay=True)
cmd_vel_avoid_sub= rospy.Subscriber("cmd_vel_avoid",    Twist,        cmd_vel_callback, tcp_nodelay=True)

# our output
cmd_vel_arb_pub =  rospy.Publisher("cmd_vel_arb",       Twist,        queue_size=5,     tcp_nodelay=True)

cmd_vel_data = Twist()
cmd_vel_data.angular.x = 0.0
cmd_vel_data.angular.y = 0.0
cmd_vel_data.linear.y =  0.0
cmd_vel_data.linear.z =  0.0

rate = rospy.Rate(10)  # needs to be slower than the slowest publisher of /cmd_vel

counter = 0

latch_duration = 1

while not rospy.is_shutdown():

    current_priority = 99
    current_command = (0.0 , 0.0)

    for c in commands.keys():
        if (priority[c] < current_priority) and (commands[c] != None):
            current_command = commands[c]
            current_priority = priority[c]
        commands[c] = None  # clear the slot....don't latch the command indefinitely


    if current_priority < 99:
        print("%8.3f: %d" % (time.monotonic(), current_priority))
        cmd_vel_data.angular.z = current_command[1] 
        cmd_vel_data.linear.x =  current_command[0]
        cmd_vel_arb_pub.publish(cmd_vel_data)   
        counter = 0
    else:
        if counter < latch_duration:
            print("%8.3f: %d  -  latched" % (time.monotonic(), current_priority))
            cmd_vel_arb_pub.publish(cmd_vel_data)
            counter  = counter + 1
        else:
            if counter == latch_duration: print("%8.3f: %d  -  stopping" % (time.monotonic(), current_priority))
            counter = latch_duration+1
            cmd_vel_data.angular.z = 0
            cmd_vel_data.linear.x =  0
            cmd_vel_arb_pub.publish(cmd_vel_data)   


    if getmtime(__file__) != file_time:
        print("restarting....");    
        cmd_vel_data.angular.x = 0.0;   cmd_vel_data.angular.y = 0.0;   cmd_vel_data.angular.z = 0.0
        cmd_vel_data.linear.x =  0.0;   cmd_vel_data.linear.y =  0.0;   cmd_vel_data.linear.z =  0.0
        cmd_vel_arb_pub.publish(cmd_vel_data);  rospy.sleep(0.5)
        rospy.signal_shutdown('restarting');    rospy.sleep(0.5);       os.execv(__file__, sys.argv)

    rate.sleep()



print("Exiting....")
