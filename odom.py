#!/usr/bin/env python3

#from a_star import AStar

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import cos, sin, pi
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
import time
import numpy as np
from i2c_testing.p2_i2c_test import AStar
import odometry
from sensor_msgs.msg import Range

print("doing imports...this could take a few seconds!")

#from odometry import odometry_update_v2

print("done with imports")

romi_control_board = AStar()
romi_control_board.motors(0, 0)

PI = 3.1415926535897932384626433832795
K_rad_to_deg = (180.0/3.1415926535897932384626433832795)
K_deg_to_rad = (PI/180.0)

'''
odo_cml = 0.15271631  # calculated based on wheel, gear and encoder specs...
odo_cmr = 0.15271631  # 0.15171631
odo_b = 148.0  # 143.0 #between 150 and 136

g_x = 0.0
g_y = 0.0
g_theta = 0.0
g_U = 0.0
g_l = 0
g_r = 0
g_dl = 0
g_dr = 0
'''
delta_L = 0
delta_R = 0
encoder_data_is_reliable = True

left_total = 0
right_total = 0
l_target = 0  # left wheel target speed - in encoder ticks per 10ms
r_target = 0
l_cmd = 0
r_cmd = 0


speed_increment = 2



def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    romi_control_board.motors(0, 0)
    #sys.exit(0)
    rospy.signal_shutdown("Ctrl+C")

#######################################################################


print("installing SIGINT handler")
signal.signal(signal.SIGINT, signal_handler)



#######################################################################


def rads(degrees):
    """
    convert from degrees to radians
    """
    return degrees * pi / 180.0


#######################################################################


def degs(radians):
    """
    convert from radians to degrees
    """
    return radians * 180 / pi


#######################################################################


def norm(theta):
    """
    normalize the angle theta so that it always falls between -pi and +pi
    """
    TWO_PI = 2.0 * pi
    normalized = theta % TWO_PI
    normalized = (normalized + TWO_PI) % TWO_PI
    if normalized > pi:
        normalized = normalized - TWO_PI
    return normalized


#######################################################################


def read_encoders():
    """
    read the encoders from the romi board and from that generate delta since the last read.
    filter out bad readings
    """
    global left_ticks, right_ticks, last_left_ticks, last_right_ticks, delta_L, delta_R, encoder_data_is_reliable

    encoder_data_is_reliable = True

    left_ticks, right_ticks = romi_control_board.read_encoders()
    if romi_control_board.error:
        left_ticks, right_ticks = romi_control_board.read_encoders()
        if romi_control_board.error:
            print("read_encoders(): double error")
            encoder_data_is_reliable = False

    delta_L = left_ticks - last_left_ticks
    delta_R = right_ticks - last_right_ticks

    if delta_L > 32000:
        delta_L = 65536 - delta_L
    if delta_L < -32000:
        delta_L = 65536 + delta_L
    if delta_R > 32000:
        delta_R = 65536 - delta_R
    if delta_R < -32000:
        delta_R = 65536 + delta_R

    # ignore faulty readings
    # maybe it would be better to repeat the last reading?
    if abs(delta_L) > 400 or abs(delta_R) > 400:
        delta_L = 0
        delta_R = 0
        # print(delta_L,delta_R)
        encoder_data_is_reliable = False
    else:
        last_left_ticks = left_ticks
        last_right_ticks = right_ticks


#######################################################################


def cmd_vel_callback(msg):
    """
    The cmd_vel_callback() gets called every time a new message of type /cmd_vel is received.
    We simply convirt it into target speed for left and right wheel
    """
    global l_cmd, r_cmd, l_target, r_target
    #rospy.loginfo("Received a /cmd_vel message!")
    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    l_target = int((msg.linear.x - msg.angular.z/14) * 68)
    r_target = int((msg.linear.x + msg.angular.z/14) * 68)


#######################################################################


def imu_callback(msg):
    odometry.th = rads(msg.point.x)


#######################################################################


ir_far_xp = [130, 145, 165, 183, 197, 210, 263, 337, 419, 576, 700, 840]
ir_far_fp = [60, 48, 40, 30, 25, 20, 15, 10, 7.5, 5, 4, 3]

ir_near_xp = [65, 112, 201, 257, 325, 467, 580, 660, 800]
ir_near_fp = [30, 20, 10, 8, 6, 4, 3, 2.5, 2]

ir_left = 0.0
ir_front= 0.0
ir_right= 0.0
us_left = 4000
us_right= 4000

def process_range_sensors():

    global ir_left,ir_front,ir_right,us_left,us_right

    analog = romi_control_board.read_analog()
    if romi_control_board.error: analog = romi_control_board.read_analog()
 
    # convert the raw ADC readings from the Sharp IR sensors into distance in mm
    ir_left  = np.interp( analog[3], ir_near_xp, ir_near_fp ) * 25.4
    ir_front = np.interp( analog[1], ir_far_xp,  ir_far_fp  ) * 25.4
    ir_right = np.interp( analog[2], ir_near_xp, ir_near_fp ) * 25.4

    # for now, the romi control board returns the sonar readings as if it were ADC readings; via index 0 and 4 (out of ADC 0....5)
    us_left  = analog[0]
    us_right = analog[4]

    # if we are getting zeros for the sonars, it means we are not getting data; probably just testing the code w/out power
    # to the romi controller;  simply set the invalid distance reading to a far out value
    if us_left  == 0:  us_left  = 4000
    if us_right == 0:  us_right = 4000

    us_left_msg = Range()

    us_left_msg.radiation_type = Range.ULTRASOUND
    us_left_msg.field_of_view = 0.7
    us_left_msg.max_range = 1.0
    us_left_msg.min_range = 0.005
    us_left_msg.range = us_left / 1000.0
    us_left_msg.header.frame_id = "base_left_us"
    us_left_msg.header.stamp = rospy.Time.now()
    
    us_left_pub.publish(us_left_msg)

    us_right_msg = Range()

    us_right_msg.radiation_type = Range.ULTRASOUND
    us_right_msg.field_of_view = 0.7
    us_right_msg.max_range = 1.0
    us_right_msg.min_range = 0.005
    us_right_msg.range = us_right / 1000.0
    us_right_msg.header.frame_id = "base_right_us"
    us_right_msg.header.stamp = rospy.Time.now()
    
    us_right_pub.publish(us_right_msg)

    ir_left_msg = Range()

    ir_left_msg.radiation_type = Range.INFRARED
    ir_left_msg.field_of_view = 0.1
    ir_left_msg.max_range = 0.6
    ir_left_msg.min_range = 0.05
    ir_left_msg.range = ir_left / 1000.0
    ir_left_msg.header.frame_id = "base_left_ir"
    ir_left_msg.header.stamp = rospy.Time.now()
    
    ir_left_pub.publish(ir_left_msg)

    ir_right_msg = Range()

    ir_right_msg.radiation_type = Range.INFRARED
    ir_right_msg.field_of_view = 0.1
    ir_right_msg.max_range = 0.6
    ir_right_msg.min_range = 0.05
    ir_right_msg.range = ir_right / 1000.0
    ir_right_msg.header.frame_id = "base_right_ir"
    ir_right_msg.header.stamp = rospy.Time.now()
    
    ir_right_pub.publish(ir_right_msg)

    ir_center_msg = Range()

    ir_center_msg.radiation_type = Range.INFRARED
    ir_center_msg.field_of_view = 0.1
    ir_center_msg.max_range = 1.5
    ir_center_msg.min_range = 0.08
    ir_center_msg.range = ir_front / 1000.0
    ir_center_msg.header.frame_id = "base_center_ir"
    ir_center_msg.header.stamp = rospy.Time.now()
    
    ir_center_pub.publish(ir_center_msg)





print("entering loop")

# Parameters
wheeltrack = 0.1417  # 0.1415
wheelradius_L = 0.035
wheelradius_R = 0.035
TPR = 1440  # 120 *12
left_ticks = 0
right_ticks = 0
last_left_ticks = 0
last_right_ticks = 0

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0

initialized = 0

rospy.init_node('romi_control_board_publisher')

odom_broadcaster = tf.TransformBroadcaster()

odom_pub =    rospy.Publisher("odom",     Odometry,     queue_size=20, tcp_nodelay=True)
encoders_pub =rospy.Publisher("encoders", PointStamped, queue_size=20, tcp_nodelay=True)
battery_pub = rospy.Publisher("battery",  Int32,        queue_size=20, tcp_nodelay=True)

us_left_pub = rospy.Publisher("us_left",  Range,        queue_size=5, tcp_nodelay=True)
us_right_pub =rospy.Publisher("us_right", Range,        queue_size=5, tcp_nodelay=True)

ir_left_pub  =rospy.Publisher("ir_left",  Range,        queue_size=5, tcp_nodelay=True)
ir_center_pub=rospy.Publisher("ir_center",Range,        queue_size=5, tcp_nodelay=True)
ir_right_pub =rospy.Publisher("ir_right", Range,        queue_size=5, tcp_nodelay=True)

cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist,        cmd_vel_callback, tcp_nodelay=True)
imu_sub =     rospy.Subscriber("imu",     PointStamped, imu_callback,     tcp_nodelay=True)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(100)

counter = 0
battery = int(romi_control_board.read_battery_millivolts())
b = battery

# initialize encoders state
read_encoders()
last_left_ticks = left_ticks
last_right_ticks = right_ticks

t_last_range_sensor_update = time.monotonic()

while not rospy.is_shutdown():

    current_time = rospy.Time.now()

    # publish the range sensors at a fixed rate of 20Hz for now
    t = time.monotonic()
    if t - t_last_range_sensor_update > 0.05:
        process_range_sensors()
        t_last_range_sensor_update = t

    # monitor timing.  doesn't seem to be an issue provided that the node
    # started with chrt -f 60
    # if current_time - last_time > rospy.Duration(0.0105):
    #  print(current_time,last_time,current_time-last_time)

    read_encoders()

    # make sure that we always start at 0,0 when this node is restarted
    # at least during testing, that makes sense
    # if not initialized:
    #    initialized = 1
    #    continue

    delta_T = (current_time - last_time).to_sec()

    odometry.odometry_update_v2(delta_L, delta_R, delta_T)

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, odometry.th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (odometry.x, odometry.y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(
        Point(odometry.x, odometry.y, 0.), Quaternion(*odom_quat))

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(
        Vector3(odometry.vx, odometry.vy, 0), Vector3(0, 0, odometry.vth))

    odom_pub.publish(odom)

    # now publish the raw encoder data - just in case we need it
    # later when playing back a rosbag
    # use msg of type PointStamped for convenience
    encoders = PointStamped()
    encoders.header.stamp = current_time
    encoders.header.frame_id = "odom"
    encoders.point.x = delta_L
    encoders.point.y = delta_R
    encoders.point.z = 0.0
    encoders_pub.publish(encoders)

    # odometry_update(delta_L,delta_R)
    # print(x,y,th,g_x,g_y,g_theta)

    if abs(delta_L - l_target) > 8:
        l_cmd_increment = 7
    else:
        l_cmd_increment = 1

    if abs(delta_R - r_target) > 8:
        r_cmd_increment = 7
    else:
        r_cmd_increment = 1

    # decellerate faster if we are trying to stop
    if l_target == 0  and  abs(delta_L - l_target) > 20 : l_cmd_increment = 15
    if r_target == 0  and  abs(delta_R - r_target) > 20 : r_cmd_increment = 15

    if delta_L > l_target:
        l_cmd -= l_cmd_increment

    if delta_L < l_target:
        l_cmd += l_cmd_increment

    if delta_R > r_target:
        r_cmd -= r_cmd_increment

    if delta_R < r_target:
        r_cmd += r_cmd_increment

    if encoder_data_is_reliable:
        # if we are trying to stop, and havel already almost stopped, then simply set the command to 0
        # so that we come to a full stop
        # but do this only if we can trust the encoder data
        if l_target == 0 and abs(delta_L) < 2:
            l_cmd = 0
        if r_target == 0 and abs(delta_R) < 2:
            r_cmd = 0

    # prevent wind-up of the commanded value
    if l_cmd >  400: l_cmd =  400
    if l_cmd < -400: l_cmd = -400
    if r_cmd >  400: r_cmd =  400
    if r_cmd < -400: r_cmd = -400

    romi_control_board.motors(l_cmd, r_cmd)
    # print(l_cmd,r_cmd)

    counter = counter+1
    if counter > 9:
        counter = 0
        time.sleep(0.0005)
        b = romi_control_board.read_battery_millivolts()
        if b > 0:
            #print("b=%5d, l_cmd=%4d, r_cmd=%4d, delta_L=%4d, delta_R=%4d, l_target=%4d, r_target=%4d x=%7.1f y=%7.1f th=%8.3f" % (int(b),l_cmd,r_cmd,delta_L,delta_R,l_target,r_target,x,y,degs(norm(th))))
            battery = (battery * 3 + b) / 4
            i = Int32()
            i.data = int(b)
            battery_pub.publish(i)
        '''
        print("t=%8.2f, b=%5d, l_cmd=%4d, r_cmd=%4d, left_t=%6d, right_t=%6d, delta_L=%4d, delta_R=%4d, l_target=%4d, r_target=%4d x=%7.3f y=%7.3f th=%8.3fdeg th=%8.3frad  us=%5d.%5d" %
            (time.monotonic(),
            int(b),
            l_cmd,
            r_cmd,
            left_ticks,
            right_ticks,
            delta_L,
            delta_R,
            l_target,
            r_target,
            odometry.x, odometry.y, degs(norm(odometry.th)),odometry.th,
            us_left, us_right))
        '''

    last_time = current_time

    t_error = delta_T - 0.01
    if True: #abs(t_error) > 0.0002:
        print("t=%9.3f, t_error=%9.6f, b=%5d, l_cmd=%4d, r_cmd=%4d, left_t=%6d, right_t=%6d, delta_L=%4d, delta_R=%4d, l_target=%4d, r_target=%4d x=%7.3f y=%7.3f th=%8.3fdeg th=%8.3frad  us=%5d.%5d" %
            (t,
            t_error,
            int(b),
            l_cmd,
            r_cmd,
            left_ticks,
            right_ticks,
            delta_L,
            delta_R,
            l_target,
            r_target,
            odometry.x, odometry.y, degs(norm(odometry.th)),odometry.th,
            us_left, us_right))

    r.sleep()




print("Exiting....")
romi_control_board.motors(0, 0)
