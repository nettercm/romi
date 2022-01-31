#!/usr/bin/env python3

print("starting scan.py")

import time, signal, sys, threading, math, os, copy
from math import cos,sin,pi

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Pose,Quaternion,Twist,Vector3,PointStamped
from std_msgs.msg import Int16, Float32MultiArray
from sensor_msgs.msg import LaserScan,PointCloud2,PointCloud

import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg

from sensor_msgs.msg import Range

import reconfiguration as r


use_minimums = True
r.params.add("use_minimums", r.bool_t, 0, "If true, use the minimum reading as the final value. If false, use the average",   True)


def config_callback(config, level):
    global use_minimums
    use_minimums            = config['use_minimums']
    return config # not sure why this is done - that's what the example did.....



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
    global skip_scan, dps, last_publish, last_hz, frame_id, laser_pub

    t = rospy.Time.now()

    l = len(scan_msg.ranges)
    valid = 0

    minimums = [99.0,99.0,99.0,99.0,99.0,99.0,99.0,99.0,99.0,99.0,99.0,99.0,99.0]
    averages = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

    hz = (3.0*last_hz + (1.0 / scan_msg.scan_time)) / 4.0
    last_hz = hz
    
    for i in range(0,12):
        s = (i * 60) - 30
        if s < 0:
            s = s + 720
        sum = 0.0
        count = 0
        for j in range(0,60):
            v = scan_msg.ranges[s]
            s = s + 1
            if s > 719:
                s = 0
            if v != float("inf"):
                valid = valid + 1
                count = count + 1
                sum = sum + v
                #use the minimum reading as the final value
                if v < minimums[i]:
                    minimums[i] = v
                #use the average reading as the final value 
                averages[i] = sum / count

    #print("%5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f " % (ranges[0],ranges[1],ranges[2],ranges[3],ranges[4],ranges[5],ranges[6],ranges[7],ranges[8],ranges[9],ranges[10],ranges[11]))
    print("%3.1fHz  len=%3d  valid=%3d   %5.3f %5.3f %5.3f   %5.3f   %5.3f %5.3f %5.3f      %5.3f %5.3f %5.3f   %5.3f   %5.3f %5.3f %5.3f  " % 
        (
            hz, l , valid, 
            minimums[3],minimums[2],minimums[1],minimums[0],minimums[11],minimums[10],minimums[9],
            averages[3],averages[2],averages[1],averages[0],averages[11],averages[10],averages[9],
        )
    )

    averages[12] = averages[0]
    minimums[12] = minimums[0]

    if use_minimums:
        values = minimums
    else:
        values = averages
        
    laser_msg = Range()
    for i in range(0,12):
        laser_msg.radiation_type = Range.INFRARED
        laser_msg.field_of_view = 0.523599  # 30 degrees
        laser_msg.max_range = 9.0
        laser_msg.min_range = 0.14
        laser_msg.range = values[12-i]
        laser_msg.header.frame_id = frame_id[i]
        laser_msg.header.stamp = scan_msg.header.stamp
        laser_pub[i].publish(laser_msg)

    ranges = Float32MultiArray(data=values)
    array_pub.publish(ranges)
    
    return




rospy.init_node('virtual_distance')

scan_sub =  rospy.Subscriber("scan",     LaserScan,    scan_callback)

laser_pub = [None,None,None,None,None,None,None,None,None,None,None,None]
frame_id  = ["laser0","laser1","laser2","laser3","laser4","laser5","laser6","laser7","laser8","laser9","laser10","laser11"]

laser_pub[0]  =rospy.Publisher("laser0",  Range,        queue_size=5, tcp_nodelay=True)
laser_pub[1]  =rospy.Publisher("laser1",  Range,        queue_size=5, tcp_nodelay=True)
laser_pub[2]  =rospy.Publisher("laser2",  Range,        queue_size=5, tcp_nodelay=True)
laser_pub[3]  =rospy.Publisher("laser3",  Range,        queue_size=5, tcp_nodelay=True)
laser_pub[4]  =rospy.Publisher("laser4",  Range,        queue_size=5, tcp_nodelay=True)
laser_pub[5]  =rospy.Publisher("laser5",  Range,        queue_size=5, tcp_nodelay=True)
laser_pub[6]  =rospy.Publisher("laser6",  Range,        queue_size=5, tcp_nodelay=True)
laser_pub[7]  =rospy.Publisher("laser7",  Range,        queue_size=5, tcp_nodelay=True)
laser_pub[8]  =rospy.Publisher("laser8",  Range,        queue_size=5, tcp_nodelay=True)
laser_pub[9]  =rospy.Publisher("laser9",  Range,        queue_size=5, tcp_nodelay=True)
laser_pub[10] =rospy.Publisher("laser10", Range,        queue_size=5, tcp_nodelay=True)
laser_pub[11] =rospy.Publisher("laser11", Range,        queue_size=5, tcp_nodelay=True)

array_pub     =rospy.Publisher("laser_array", Float32MultiArray, queue_size=5, tcp_nodelay=True)

TWO_PI = 2.0 * pi

start_time = rospy.Duration(0)
time_offset = rospy.Time.now()

r.start(config_callback)

while not rospy.is_shutdown():
    rospy.spin()
    
    
