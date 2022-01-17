#!/usr/bin/env python3

print("starging scan_filter.py");

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


dps = 0.0

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



def odom_callback(msg):

    #if start_time == 0:
    #    start_time = msg.header.stamp

    return	    


skip_scan = 1
last_publish = rospy.Time(0)

def scan_callback(scan_msg : LaserScan):
    global skip_scan, dps, last_publish

    t = rospy.Time.now()
    dT = t-last_publish
    dT = dT.to_sec()

    if dT < 0:
        print("time went backwards....!")
        last_publish = t

    if dps < 65.0:
        if dT > 0.0:
            scan_pub.publish(scan_msg)
            last_publish = t

    return



def imu_callback(imu_msg : PointStamped):
    global dps
    #print(imu_msg.point.z)
    dps = imu_msg.point.z
    return 

rospy.init_node('scan_filter')

odom_sub =  rospy.Subscriber("odom",     Odometry,     odom_callback)
scan_sub =  rospy.Subscriber("scan",     LaserScan,    scan_callback)
imu_sub  =  rospy.Subscriber("imu",      PointStamped, imu_callback)

scan_pub =  rospy.Publisher("scan_2",    LaserScan,    queue_size=5, tcp_nodelay=True) 

TWO_PI = 2.0 * pi

start_time = rospy.Duration(0)
time_offset = rospy.Time.now()



while not rospy.is_shutdown():
    rospy.spin()
    
    
