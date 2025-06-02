#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import signal
import sys

class CmdVelMonitor(Node):
    def __init__(self):
        super().__init__('cmd_vel_monitor')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10  # QoS profile depth
        )
        self.last_linear_x = None
        self.last_angular_z = None
        self.get_logger().info('CmdVelMonitor started - watching for cmd_vel messages')

    def cmd_vel_callback(self, msg):
        current_time = time.strftime("%H:%M:%S", time.localtime())
        
        # Check if linear.x or angular.z have changed
        if self.last_linear_x != msg.linear.x or self.last_angular_z != msg.angular.z:
            self.get_logger().info(f"[{current_time}] linear.x: {msg.linear.x:.4f}, angular.z: {msg.angular.z:.4f}")
            
            # Update the last known values
            self.last_linear_x = msg.linear.x
            self.last_angular_z = msg.angular.z

def signal_handler(sig, frame):
    print('Ctrl+C pressed, shutting down...')
    # Using sys.exit directly ensures immediate termination
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    
    node = CmdVelMonitor()
    
    # Register signal handler for Ctrl+C - simplified to not pass the node
    #signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('KeyboardInterrupt caught in spin()')
    finally:
        print('Cleaning up...')
        # Clean up
        time.sleep(0.1)
        try:
            node.destroy_node()
        except Exception as e:
            print(f"Error destroying node: {e}")
            
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during ROS shutdown: {e}")
            
        print('Exiting normally')

if __name__ == '__main__':
    main()
