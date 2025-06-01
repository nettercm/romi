#!/usr/bin/env python3

import time
import signal
import sys
import threading
import termios
import fcntl
import os
from math import cos, sin, pi

# ROS 2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
from geometry_msgs.msg import PointStamped

import qtpy

done = False

def signal_handler(sig, frame):
    global done
    print('You pressed Ctrl+C!')
    done = True
    #rclpy.shutdown()  # Make sure ROS shuts down


def odom_reset_callback(msg):
    current_time = time.monotonic()
    print(f"odom reset at timestamp: {current_time:.6f}!")
    qtpy.heading_calib = 0.0
    qtpy.heading_delta_calib_accumulated = 0.0
    qtpy.dps = 0.0
    qtpy.dps_max = 0.0
    return

signal.signal(signal.SIGINT, signal_handler)

rclpy.init()
node = rclpy.create_node('qtpy_publisher')

imu_pub = node.create_publisher(PointStamped, 'imu', 5)
line_pub = node.create_publisher(Int32MultiArray, 'line', 5)
reset_sub = node.create_subscription(Int32, 'odom_reset', odom_reset_callback, 5)

qtpy.initialize()

# Set target loop frequency
loop_hz = 100  # 100 Hz
loop_period = (1.0 / loop_hz) - 0.0002  # Allow a small buffer for processing time

# Initialize counter for spin_once calls
iteration_counter = 0

while not done and rclpy.ok():
    start_time = time.monotonic()
    #print(start_time)
    
    # Process any pending callbacks (non-blocking) only once every 10 iterations
    iteration_counter += 1
    if iteration_counter >= 10:
        rclpy.spin_once(node, timeout_sec=0.0)  # Set to 0 to make it truly non-blocking
        iteration_counter = 0

    result = False
    result = qtpy.update()
    if result:
        current_time = node.get_clock().now().to_msg()
        imu_data = PointStamped()
        imu_data.header.stamp = current_time
        imu_data.header.frame_id = "base_link"
        imu_data.point.x = qtpy.heading_calib
        imu_data.point.y = qtpy.heading_delta_calib_accumulated
        imu_data.point.z = qtpy.dps
        imu_pub.publish(imu_data)
        line_data = Int32MultiArray(data=qtpy.line)
        line_pub.publish(line_data)
    else:
        #print(time.monotonic(), "qtpy.update() returned False, skipping publishing")
        pass
        
    # Calculate time to sleep and sleep only if needed
    elapsed = time.monotonic() - start_time
    sleep_time = loop_period - elapsed
    if True:
        if sleep_time > 0:
            #print(f"Sleeping for {sleep_time:.4f} seconds")
            time.sleep(sleep_time)
        else:
            # If we're running behind, just yield to the OS briefly
            time.sleep(0.001)
    else:
        # If we're not sleeping, just yield to the OS briefly
        time.sleep(0.001)
        
time.sleep(0.2)  # Give some time for the last messages to be sent
qtpy.deinitialize()
time.sleep(0.2)  # Give some time for the last messages to be sent
node.destroy_node()
