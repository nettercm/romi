#!/usr/bin/env python2

#from a_star import AStar

print("doing imports...this could take a few seconds!")

from i2c_testing.p2_i2c_test import AStar
import time
import signal
import sys
import threading
import math
from math import cos,sin,pi
import termios
import fcntl
import os
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Pose,Quaternion,Twist,Vector3
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int16,Int32

print("done with imports")

a_star = AStar()
a_star.motors(0, 0);

PI=3.1415926535897932384626433832795;
K_rad_to_deg=(180.0/3.1415926535897932384626433832795);
K_deg_to_rad=(PI/180.0);

odo_cml=0.15271631; #calculated based on wheel, gear and encoder specs...
odo_cmr=0.15271631  #0.15171631;
odo_b=148.0; #143.0; #between 150 and 136

g_x=0.0;
g_y=0.0;
g_theta=0.0;
g_U=0.0;
g_l=0;
g_r=0;
g_dl=0;
g_dr=0;

left_total=0;
right_total=0;
l_target = 0;
r_target = 0;
l_cmd = 0;
r_cmd = 0;

speed_increment = 2;

'''
#######################################################################

from pynput import keyboard

def on_press(key):
  global l_cmd,r_cmd,l_target,r_target,speed_increment;
  #print('You pressed a key')
  if key == keyboard.Key.up:
    l_target += speed_increment;
    r_target += speed_increment;
  if key == keyboard.Key.down:
    l_target -= speed_increment;
    r_target -= speed_increment;
  if key == keyboard.Key.left:
    l_target -= speed_increment;
    r_target += speed_increment;
  if key == keyboard.Key.right:
    l_target += speed_increment;
    r_target -= speed_increment;
  if key == keyboard.Key.space:
    l_target = 0;
    r_target = 0;
    l_cmd = 0;
    r_cmd = 0;
  return True;
    
#######################################################################
    
def on_release(key):
    #print('You released a key')
    if key == keyboard.Key.esc:
      print('You pressed Esc')
      #return False
    return True

#######################################################################

print("starting keyboard listener")
listener = keyboard.Listener( on_press=on_press, on_release=on_release )
listener.start()


'''

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    a_star.motors(0, 0);
    sys.exit(0)

#######################################################################

def debug_thread(name):
  global g_l,g_r,g_dl,g_dr;
  while 1:
    print(g_x,g_y, K_rad_to_deg*g_theta);
    time.sleep(0.2);
    
#######################################################################
    
def odometry_update(l_ticks, r_ticks):
#  float d_theta, d_x, d_y, l, r;
#  float d_Ul, d_Ur, d_U;
  global g_x;
  global g_y;
  global g_theta;
  global g_U;
  global g_l;
  global g_r;
  global g_dl;
  global g_dr;

  g_l += l_ticks;
  g_r += r_ticks;
  g_dl = l_ticks;
  g_dr = r_ticks;
  
  l = float(l_ticks);
  r = float(r_ticks);

  d_Ul = odo_cml * l;
  d_Ur = odo_cmr * r;
  d_U  = (d_Ul + d_Ur) / 2.0;
  d_theta = (d_Ur - d_Ul) / odo_b;

  #update our absolute position
  d_x = d_U * cos(g_theta);
  d_y = d_U * sin(g_theta);
  g_x  = g_x + d_x;
  g_y  = g_y + d_y;
  g_theta = g_theta + d_theta;
  g_U      = g_U + d_U;

  if g_theta > 1.0*PI:
    g_theta -= 2.0*PI;
  #if(g_theta < -2.0*PI) g_theta += 2.0*PI;
  if g_theta < -1.0*PI:
    g_theta += 2.0*PI;

#######################################################################

def speed_control_thread(name):
  global l_cmd,r_cmd,l_target,r_target,left_total,right_total;
  left1,right1 = a_star.read_encoders();
  while 1:
    time.sleep(0.02);  
    
    #first let's read...
    left2,right2 = a_star.read_encoders();
    #analog = a_star.read_analog();
    
    left_delta=left2-left1;
    if left_delta > 32000:
      left_delta = 65536 - left_delta;
    if left_delta < -32000:
      left_delta = 65536 + left_delta;
    right_delta=right2-right1;
    if right_delta > 32000:
      right_delta = 65536 - right_delta;
    if right_delta < -32000:
      right_delta = 65536 + right_delta;
    left_total += left_delta;
    right_total += right_delta;
    left1=left2;
    right1=right2;
    odometry_update(left_delta,right_delta);

    if left_delta > l_target:
      l_cmd -= 1;
    if left_delta < l_target:
      l_cmd += 1;
    if right_delta > r_target:
      r_cmd -= 1;
    if right_delta < r_target:
      r_cmd += 1;

    #now let's write
    a_star.motors(l_cmd, r_cmd);
  
#######################################################################

print("installing SIGINT handler")
signal.signal(signal.SIGINT, signal_handler)

"""
print("starting debug_thread")
t1=threading.Thread(target=debug_thread, args=(1,));
t1.daemon=True;
t1.start();

print("starting speed_control_thread")
t2=threading.Thread(target=speed_control_thread, args=(1,));
t2.daemon=True;
t2.start();


"""





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




def cmd_vel_callback(msg):
    global l_cmd,r_cmd,l_target,r_target
    #rospy.loginfo("Received a /cmd_vel message!")
    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    #l_cmd = int((msg.linear.x - msg.angular.z) * 100)
    #r_cmd = int((msg.linear.x + msg.angular.z) * 100)
    l_target = int((msg.linear.x - msg.angular.z) * 100)
    l_cmd = l_target
    r_target = int((msg.linear.x + msg.angular.z) * 100)
    r_cmd = r_target
    #print(l_cmd,r_cmd)
    
   
print("entering loop")

#Parameters
wheeltrack = 0.1417 #0.1415
wheelradius_L = 0.035
wheelradius_R = 0.035
TPR = 1440 #120 *12
left_ticks = 0
right_ticks = 0
last_left_ticks = 0
last_right_ticks = 0

x = 0.0
y = 0.0
th = 0.0

vx =  0.0
vy =  0.0
vth =  0.0

initialized = 0

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50, tcp_nodelay=True)
odom_broadcaster = tf.TransformBroadcaster()

encoders_pub = rospy.Publisher("encoders", PointStamped, queue_size=50, tcp_nodelay=True)
battery_pub = rospy.Publisher("battery", Int32, queue_size=50, tcp_nodelay=True)
cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback, tcp_nodelay=True)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(100)

counter = 0
battery = int( a_star.read_battery_millivolts() )
b = battery 

left_ticks,right_ticks = a_star.read_encoders()
if a_star.error:
  left_ticks,right_ticks = a_star.read_encoders()
  if a_star.error:
    print("read_encoders(): double error")
last_left_ticks = left_ticks
last_right_ticks = right_ticks

while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    
    #monitor timing.  doesn't seem to be an issue provided that the node
    #started with chrt -f 60
    #if current_time - last_time > rospy.Duration(0.0105):
    #  print(current_time,last_time,current_time-last_time)
    left_ticks,right_ticks = a_star.read_encoders()
    if a_star.error:
      left_ticks,right_ticks = a_star.read_encoders()
      if a_star.error:
        print("read_encoders(): double error")

    delta_L = left_ticks - last_left_ticks
    delta_R = right_ticks - last_right_ticks

    if delta_L > 32000:
      delta_L = 65536 - delta_L;
    if delta_L < -32000:
      delta_L = 65536 + delta_L;
    if delta_R > 32000:
      delta_R = 65536 - delta_R;
    if delta_R < -32000:
      delta_R = 65536 + delta_R;

    #ignore faulty readings
    if abs(delta_L > 200) or abs(delta_R > 200):
      delta_L = 0
      delta_R = 0
    else:
      last_left_ticks = left_ticks
      last_right_ticks = right_ticks
    
    last_time = current_time

    #make sure that we always start at 0,0 when this node is restarted
    #at least during testing, that makes sense
    if not initialized:
        initialized = 1
        continue
        
    dl = 2 * pi * wheelradius_L * delta_L / TPR
    dr = 2 * pi * wheelradius_R * delta_R / TPR
    dc = (dl + dr) / 2
    dt = (current_time - last_time).to_sec()
    dth = (dr-dl)/wheeltrack
    
    if dr==dl:
        dx=dr*cos(th)
        dy=dr*sin(th)

    else:
        radius=dc/dth
        iccX=x-radius*sin(th)
        iccY=y+radius*cos(th)
        dx = cos(dth) * (x-iccX) - sin(dth) * (y-iccY) + iccX - x
        dy = sin(dth) * (x-iccX) + cos(dt) * (y-iccY) + iccY - y

    x += dx  
    y += dy 
    th =(th+dth) %  (2 * pi)

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
       (x, y, 0.),
       odom_quat,
       current_time,
       "base_link",
       "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    if dt>0:
       vx=dx/dt
       vy=dy/dt
       vth=dth/dt

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    odom_pub.publish(odom)
    
    #now publish the raw encoder data - just in case we need it
    #later when playing back a rosbag
    #use msg of type PointStamped for convenience 
    encoders=PointStamped()
    encoders.header.stamp = current_time
    encoders.header.frame_id = "odom"
    encoders.point.x = delta_L
    encoders.point.y = delta_R
    encoders.point.z = 0.0
    encoders_pub.publish(encoders)

    #odometry_update(delta_L,delta_R)
    #print(x,y,th,g_x,g_y,g_theta)
    
    if delta_L > l_target:
      l_cmd -= 1;
    if delta_L < l_target:
      l_cmd += 1;
    if delta_R > r_target:
      r_cmd -= 1;
    if delta_R < r_target:
      r_cmd += 1;

    a_star.motors(l_cmd, r_cmd)
    #print(l_cmd,r_cmd)
    
    counter = counter+1
    if counter > 9:
      counter = 0
      time.sleep(0.0005)
      b = a_star.read_battery_millivolts()
      if b > 0:
        #print("b=%5d, l_cmd=%4d, r_cmd=%4d, delta_L=%4d, delta_R=%4d, l_target=%4d, r_target=%4d x=%7.1f y=%7.1f th=%8.3f" % (int(b),l_cmd,r_cmd,delta_L,delta_R,l_target,r_target,x,y,degs(norm(th))))
        battery = (battery * 3 + b) / 4
        i = Int32()
        i.data = int(b)
        battery_pub.publish(i)
      
    print("b=%5d, l_cmd=%4d, r_cmd=%4d, left_t=%6d, right_t=%6d, delta_L=%4d, delta_R=%4d, l_target=%4d, r_target=%4d x=%7.1f y=%7.1f th=%8.3f" % (int(b),l_cmd,r_cmd,left_ticks,right_ticks,delta_L,delta_R,l_target,r_target,x,y,degs(norm(th))))
  
    r.sleep()
