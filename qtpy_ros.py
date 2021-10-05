#!/usr/bin/env python2

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

# qtpy "driver"
import qtpy


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


rospy.init_node('qtpy_publisher')

imu_pub = rospy.Publisher("imu", PointStamped, queue_size=5, tcp_nodelay=True)
line_pub = rospy.Publisher("line", Int32MultiArray, queue_size=5, tcp_nodelay=True)

current_time = last_time = rospy.Time.now()

# initialize the qtpy "driver" module - this will open the serial port, etc. etc.
qtpy.initialize()

# the main loop rate will be governed by the rate at which the qtpy driver provides data (100Hz), 
# hence the loop body does not contain any form of a delay or rate limiting statement
while (not done) and (not rospy.is_shutdown()):

    # get new data from the qtpy - this is a blocking call with 10ms timeout
    result = qtpy.update()

    if result != True:
        # 99 out of 100 times there will be new data at this point, because the call to .update() is semi-blocking
        # but if there was not new data, simply skip the rest of this loop
        continue

    # for timestamping purposes....
    current_time = rospy.Time.now()

    # we might need this timestamp too....works better for print() statements
    t = time.monotonic()

    #print("%8d, %7.3f, %8.4f, %8.4f, %11.2f, dps=%6.2f" % (qtpy.timestamp, t,
    #    qtpy.heading_calib, qtpy.heading_uncal_new, qtpy.heading_delta_calib_accumulated, qtpy.dps))

    # publish the imu data via the standard message type "PointStamped", because it contains room for 3 floats and a timestamp
    # could be using the actual message type that is intended for IMU data here, but that type is a bit more convoluted 
    imu_data = PointStamped()
    imu_data.header.stamp = current_time
    imu_data.header.frame_id = "base_link"
    imu_data.point.x = qtpy.heading_calib
    imu_data.point.y = qtpy.heading_delta_calib_accumulated
    imu_data.point.z = qtpy.dps
    imu_pub.publish(imu_data)

    # publish the line sensor array data as simply one-dimensional array
    line_data = Int32MultiArray(data=qtpy.line)
    line_pub.publish(line_data)
   

qtpy.deinitialize()
