#!/usr/bin/env python2

# standard imports
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

# qtpy "driver"
import qtpy



done = False


def signal_handler(sig, frame):
    global done
    print('You pressed Ctrl+C!')
    sys.exit(0)
    done = True
    qtpy.done = True

signal.signal(signal.SIGINT, signal_handler)

rospy.init_node('qtpy_publisher')

imu_pub = rospy.Publisher("imu", PointStamped, queue_size=5, tcp_nodelay=True)
line_pub = rospy.Publisher("line", Int32MultiArray, queue_size=5, tcp_nodelay=True)

current_time = last_time = rospy.Time.now()

r = rospy.Rate(100)

qtpy.initialize()

while not rospy.is_shutdown():

    current_time = rospy.Time.now()

    result = qtpy.update()

    if result != True:
        continue

    t = time.monotonic()

    #print("%8d, %7.3f, %8.4f, %8.4f, %11.2f, dps=%6.2f" % (qtpy.timestamp, t,
    #    qtpy.heading_calib, qtpy.heading_uncal_new, qtpy.heading_delta_calib_accumulated, qtpy.dps))

    imu_data = PointStamped()
    imu_data.header.stamp = current_time
    imu_data.header.frame_id = "base_link"
    imu_data.point.x = qtpy.heading_calib
    imu_data.point.y = qtpy.heading_delta_calib_accumulated
    imu_data.point.z = qtpy.dps
    imu_pub.publish(imu_data)

    line_data = Int32MultiArray(data=qtpy.line)
    line_pub.publish(line_data)
   
