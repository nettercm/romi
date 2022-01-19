#!/usr/bin/env python3

print("starging scan.py");

import time, signal, sys, threading, math, os, copy
from math import cos,sin,pi

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Pose,Quaternion,Twist,Vector3,PointStamped
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan,PointCloud2,PointCloud

import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg


def rads(degrees):
    """
    convert from degrees to radians
    """
    return degrees * pi / 180.0



def degs(radians):
    """
    convert from radians to degrees
    """
    return radians * 180 / pi



def norm(theta):
    """
    normalize the angle theta so that it always falls between -pi and +pi
    """
    TWO_PI = 2.0 * pi
    normalized = theta % TWO_PI;
    normalized = (normalized + TWO_PI) % TWO_PI;
    if normalized > pi:
        normalized = normalized - TWO_PI;
    return normalized



def euler_angle_from_pose(pose):
    """
    extract the angle theta from the x,y,z,w quaternion that is part of pose structure
    return theta
    """
    quaternion = ( pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    th= euler[2]
    th= norm(th)
    return th



def ms(time):
    global start_time
    t = (time - start_time).to_nsec() / 100000
    t = int(t)
    t = float(t/10.0)
    return t


last_publish = rospy.Time(0)
last_hz = 0

def scan_callback(scan_msg : LaserScan):
    global skip_scan, dps, last_publish, last_hz

    t = rospy.Time.now()

    l = len(scan_msg.ranges)
    valid = 0

    ranges = [9,9,9,9,9,9,9,9,9,9,9,9]

    hz = (3.0*last_hz + (1.0 / scan_msg.scan_time)) / 4.0
    last_hz = hz
    
    for i in range(0,12):
        s = (i * 60) - 30
        if s < 0:
            s = s + 720
        for j in range(0,60):
            v = scan_msg.ranges[s]
            s = s + 1
            if s > 719:
                s = 0
            if v != float("inf"):
                valid = valid + 1
                if v < ranges[i]:
                    ranges[i] = v

    #print("%5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f " % (ranges[0],ranges[1],ranges[2],ranges[3],ranges[4],ranges[5],ranges[6],ranges[7],ranges[8],ranges[9],ranges[10],ranges[11]))
    print("%3.1fHz  len=%3d  valid=%3d   %5.3f %5.3f %5.3f   %5.3f   %5.3f %5.3f %5.3f  " % (hz, l , valid, ranges[3],ranges[2],ranges[1],ranges[0],ranges[11],ranges[10],ranges[9]))

    return




rospy.init_node('scan')

scan_sub =  rospy.Subscriber("scan",     LaserScan,    scan_callback)


TWO_PI = 2.0 * pi

start_time = rospy.Duration(0)
time_offset = rospy.Time.now()



while not rospy.is_shutdown():
    rospy.spin()
    
    
