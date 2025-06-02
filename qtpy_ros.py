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

# Global flags for thread coordination
done = False
ros_thread_running = False
qtpy_lock = threading.Lock()  # Lock for thread-safe qtpy operations
shutdown_lock = threading.Lock()  # Lock for coordinating shutdown
shutdown_called = False

def signal_handler(sig, frame):
    global done
    print('You pressed Ctrl+C!')
    done = True


def odom_reset_callback(msg):
    current_time = time.monotonic()
    print(f"odom reset at timestamp: {current_time:.6f}!")
    # Thread-safe access to qtpy variables
    with qtpy_lock:
        qtpy.heading_calib = 0.0
        qtpy.heading_delta_calib_accumulated = 0.0
        qtpy.dps = 0.0
        qtpy.dps_max = 0.0
    return


def ros_spin_thread(node):
    """Dedicated thread for handling ROS callbacks and spinning"""
    global ros_thread_running, done
    ros_thread_running = True
    print("ROS spin thread started")
    
    try:
        while not done and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as e:
        print(f"ROS spin thread error: {e}")
    finally:
        ros_thread_running = False
        print("ROS spin thread stopped")


def safe_shutdown():
    """Thread-safe shutdown function"""
    global shutdown_called
    with shutdown_lock:
        if shutdown_called:
            return
        shutdown_called = True
        
        print("Performing safe shutdown...")
        
        # Clean up qtpy
        try:
            qtpy.deinitialize()
        except Exception as e:
            print(f"Error during qtpy deinitialize: {e}")
        
        # Clean up ROS node
        try:
            if 'node' in globals():
                node.destroy_node()
        except Exception as e:
            print(f"Error destroying node: {e}")
        
        # Only shutdown ROS if it's still running
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during ROS shutdown: {e}")
        
        print("Shutdown completed")

signal.signal(signal.SIGINT, signal_handler)

rclpy.init()
node = rclpy.create_node('qtpy_publisher')

imu_pub = node.create_publisher(PointStamped, 'imu', 5)
line_pub = node.create_publisher(Int32MultiArray, 'line', 5)
reset_sub = node.create_subscription(Int32, 'odom_reset', odom_reset_callback, 5)

qtpy.initialize()

# Start the ROS spinning thread
ros_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
ros_thread.start()


print("Starting main sensor loop")

while not done and rclpy.ok():
    
    # Update sensors with thread-safe access
    with qtpy_lock:
        result = qtpy.update() # non-blocking update of qtpy sensors
        if result:
            # Copy data while holding the lock
            heading_calib = qtpy.heading_calib
            heading_delta_calib_accumulated = qtpy.heading_delta_calib_accumulated
            dps = qtpy.dps
            line_data_copy = qtpy.line.copy() if hasattr(qtpy.line, 'copy') else list(qtpy.line)
    
    # Publish data outside of the lock
    if result:
        current_time = node.get_clock().now().to_msg()
        imu_data = PointStamped()
        imu_data.header.stamp = current_time
        imu_data.header.frame_id = "base_link"
        imu_data.point.x = heading_calib
        imu_data.point.y = heading_delta_calib_accumulated
        imu_data.point.z = dps
        if not done: imu_pub.publish(imu_data)
        line_data = Int32MultiArray(data=line_data_copy)
        if not done: line_pub.publish(line_data)
        #time.sleep(0.003)
    else:
        #time.sleep(0.001)  # Sleep briefly if no update was made
        pass
        

print("Main loop ended, cleaning up...")

# Wait for ROS thread to finish
if ros_thread.is_alive():
    print("Waiting for ROS thread to finish...")
    ros_thread.join(timeout=2.0)
    if ros_thread.is_alive():
        print("ROS thread did not finish gracefully")

# Perform safe shutdown
safe_shutdown()
