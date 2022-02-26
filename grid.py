#!/usr/bin/python3

import rospy
import numpy
import math
import tf
from sensor_msgs.msg import LaserScan,PointCloud2,PointCloud
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped

from bresenham import bresenham

map_msg = OccupancyGrid()
map_msg.header.frame_id = 'map'
resolution = 0.025
width = 400
height = 400

dps = 0.0

map_msg.info.resolution = resolution
map_msg.info.width = width
map_msg.info.height = height
map_msg.data = [0] * width * height

# set map origin [meters]
map_msg.info.origin.position.x = - width // 2 * resolution
map_msg.info.origin.position.y = - height // 2 * resolution

for i in range(width*height):
    map_msg.data[i] = -1


def clear_grid(x,y):
    y = y + 200
    x = x + 200
    i = y*width+x
    if map_msg.data[i] > 0:
         map_msg.data[i] =  map_msg.data[i] - 2


def set_grid(x,y):
    y = y + 200
    x = x + 200
    i = y*width+x
    if map_msg.data[i] < 100:
         map_msg.data[i] =  map_msg.data[i] + 10



def pc_callback(msg: PointCloud):
    if dps > 15:
        return
    l = len(msg.points)
    #for i in range(width*height):
    #    map_msg.data[i] = -1
    for i in range(0,l):
        x = int(msg.points[i].x / resolution - 0.5)
        y = int(msg.points[i].y / resolution - 0.5)
        #y = y + 200
        #x = x + 200
        #map_msg.data[y*width+x] = 100
        cells = list(bresenham(0,0,x,y))
        lc = len(cells)
        for j in range(0,lc):
            xc,yc = cells[j]
            clear_grid(xc,yc)
        set_grid(xc,yc)
    #print("%d,%d,%d" % (l,x,y))
    return



def imu_callback(msg: PointStamped):
    global dps # degrees per second
    dps = abs(msg.point.z)


rospy.init_node('grid')

occ_pub = rospy.Publisher("/map", OccupancyGrid, queue_size = 1, tcp_nodelay=True)

pc_sub =  rospy.Subscriber("/my_cloud",     PointCloud,    pc_callback)

imu_sub =     rospy.Subscriber("imu",     PointStamped, imu_callback,     tcp_nodelay=True)

rate = 2.0

loop_rate = rospy.Rate(rate)

while not rospy.is_shutdown():

    print("%6.1f" % rospy.Time.now().to_sec())

    occ_pub.publish(map_msg)
    loop_rate.sleep()
