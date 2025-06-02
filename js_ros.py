#!/usr/bin/env python3

import time
import signal
import sys
import os
from math import cos, sin, pi
from dataclasses import dataclass

# ROS2 imports
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult

# "driver"
import js_linux as js
#import reconfiguration as r  # You may need to update this for ROS2 or replace with parameter handling
from utilities import *

from os.path import getmtime
file_time = getmtime(__file__)

@dataclass
class JoystickState:
    max_linear_velocity: float
    max_angular_velocity: float
    linear_velocity_slew_rate: float
    angular_velocity_slew_rate: float
    auto_idle: bool
    idle_timeout: float
    current_vx: float = 0.0
    current_vth: float = 0.0
    joystick_is_idle: bool = False
    t_idle: float = 0.0
    done: bool = False
    file_time: float = 0.0
    joy_pub: object = None  # Publisher

# This function is called whenever a new Odometry message is received on the 'odom_slow' topic.
# It updates the robot's current linear and angular velocity in the shared state dictionary.
def odom_callback(msg, state: JoystickState):
    state.current_vx = msg.twist.twist.linear.x
    state.current_vth = msg.twist.twist.angular.z



# This function is called periodically (every 0.05 seconds, i.e., 20 Hz) by a ROS2 timer.
# It reads joystick input, computes the desired velocities, handles idle logic, and publishes Twist messages.
def main_loop(node, state: JoystickState):
    # If the program is marked as done or ROS2 is shutting down, clean up and exit the loop.
    if state.done or not rclpy.ok():
        return

    # Read joystick axes for movement commands
    y = -js.axis_states['y']  # Forward/backward
    rx = -js.axis_states['z'] #['rx']  # Rotation

    # Scale joystick input to velocity using parameters
    js_linear_velocity = y * state.max_linear_velocity
    js_angular_velocity = rx * state.max_angular_velocity

    # Idle detection: if joystick is untouched and 'a' button is not pressed
    if (abs(js_angular_velocity) < 0.01) and (abs(js_linear_velocity) < 0.01) and js.button_states['a'] == 0:
        state.t_idle += 0.05  # Increment idle timer
        if (state.t_idle > state.idle_timeout) and (state.auto_idle):
            if not state.joystick_is_idle:
                node.get_logger().info("joystick idle")
            state.joystick_is_idle = True
    else:
        if state.joystick_is_idle:
            node.get_logger().info("joystick is busy")
        state.t_idle = 0.0
        state.joystick_is_idle = False

    # If 'b' button is pressed, immediately set joystick to idle
    if js.button_states['b'] == 1:
        if not state.joystick_is_idle:
            node.get_logger().info("joystick idle")
        state.joystick_is_idle = True

    # Adjust stopping behavior for smooth deceleration
    linear_stopping_multiplier = 1.0
    angular_stopping_multiplier = 1.0
    if abs(js_angular_velocity) < 0.05 or js.button_states['a'] == 1:
        angular_stopping_multiplier = 1.8
    if abs(js_linear_velocity) < 0.05 or js.button_states['a'] == 1:
        linear_stopping_multiplier = 1.4

    # Prepare the Twist message to send to the robot
    cmd_vel_data = Twist()
    cmd_vel_data.angular.x = 0.0
    cmd_vel_data.angular.y = 0.0
    cmd_vel_data.angular.z = slew(state.current_vth, js_angular_velocity, state.angular_velocity_slew_rate * angular_stopping_multiplier)
    cmd_vel_data.linear.x = slew(state.current_vx, js_linear_velocity, state.linear_velocity_slew_rate * linear_stopping_multiplier)
    cmd_vel_data.linear.y = 0.0
    cmd_vel_data.linear.z = 0.0

    # Only publish if joystick is not idle
    if not state.joystick_is_idle:
        state.joy_pub.publish(cmd_vel_data)

    # If the script file has changed, restart the process (for hot-reloading during development)
    if getmtime(__file__) != state.file_time:
        node.get_logger().info("restarting....")
        # First set the done flag in js_linux to stop the joystick thread
        js.done = True
        # Then set our state's done flag
        state.done = True
        # Wait a moment for the js thread to acknowledge the done flag
        time.sleep(0.1)
        # Force ROS to exit the spin loop
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"Error shutting down ROS during restart: {e}")
        return  # Exit the timer loop gracefully



# This function is called when Ctrl+C is pressed in the terminal.
# It sets the 'done' flag and shuts down ROS2 cleanly.
def signal_handler(sig, frame, state: JoystickState):
    print('You pressed Ctrl+C!')
    # First set the done flag in js_linux to stop the joystick thread
    js.done = True
    # Then set our state's done flag
    state.done = True
    # Wait a moment for the js thread to acknowledge the done flag
    time.sleep(0.1)
    # Signal ROS to stop spinning by shutting down rclpy
    if rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Error shutting down ROS: {e}")



def parameter_callback(params, state: JoystickState):
    for param in params:
        if param.name == 'max_linear_velocity':
            state.max_linear_velocity = param.value
        elif param.name == 'max_angular_velocity':
            state.max_angular_velocity = param.value
        elif param.name == 'linear_velocity_slew_rate':
            state.linear_velocity_slew_rate = param.value
        elif param.name == 'angular_velocity_slew_rate':
            state.angular_velocity_slew_rate = param.value
        elif param.name == 'auto_idle':
            state.auto_idle = param.value
        elif param.name == 'idle_timeout':
            state.idle_timeout = param.value
    return rclpy.parameter.ParameterEventHandler.Result(successful=True)



def on_parameter_event(params, state: JoystickState):
    for param in params:
        if hasattr(state, param.name):
            setattr(state, param.name, param.value)
    return SetParametersResult(successful=True)



# The main function sets up the ROS2 node, publishers, subscribers, parameters, and timers.
def main(args=None):
    # Initialize ROS2 Python client library
    rclpy.init(args=args)
    # Create a node instance (not a subclass, just a plain node)
    node = rclpy.create_node('joystick')

    # Declare and get parameters (these can be set from the command line or launch file)
    node.declare_parameter('max_linear_velocity', 0.7)
    node.declare_parameter('max_angular_velocity', 6.28)
    node.declare_parameter('linear_velocity_slew_rate', 0.22)
    node.declare_parameter('angular_velocity_slew_rate', 5.6)
    node.declare_parameter('auto_idle', True)
    node.declare_parameter('idle_timeout', 3.0)

    # Create the state object
    state = JoystickState(
        max_linear_velocity=node.get_parameter('max_linear_velocity').value,
        max_angular_velocity=node.get_parameter('max_angular_velocity').value,
        linear_velocity_slew_rate=node.get_parameter('linear_velocity_slew_rate').value,
        angular_velocity_slew_rate=node.get_parameter('angular_velocity_slew_rate').value,
        auto_idle=node.get_parameter('auto_idle').value,
        idle_timeout=node.get_parameter('idle_timeout').value,
        file_time=getmtime(__file__),
        joy_pub=node.create_publisher(Twist, 'cmd_vel', 5),
    )
    print(state)

    # Initialize the joystick hardware/driver
    js.js_init()

    # Create a subscriber for odometry messages.
    node.create_subscription(
        Odometry,                # Message type
        'odom_slow',             # Topic name
        lambda msg: odom_callback(msg, state), # Callback function (calls odom_callback with state)
        5                        # Queue size
    )

    # Create a timer that calls main_loop every 0.05 seconds (20 Hz).
    timer = node.create_timer(0.05, lambda: main_loop(node, state))

    # Register a signal handler for Ctrl+C (SIGINT) to allow clean shutdown.
    #signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, state))

    # Add parameter callback using lambda to pass state
    node.add_on_set_parameters_callback(lambda params: on_parameter_event(params, state))

    # Start the ROS2 event loop. This will keep the program running, calling callbacks as needed.
    restart_requested = False
    cleanup_done = False
    try:
        rclpy.spin(node)  # This will process incoming messages and timer events
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        # Set the done flag in js_linux to stop the joystick thread
        js.done = True
        # Then set our state's done flag
        state.done = True
        # Wait a moment for the js thread to acknowledge the done flag
        time.sleep(0.1)
    finally:
        if not cleanup_done:
            print("Cleaning up...")
            
            cleanup_done = True
            
            # Check if restart was requested due to file change
            if state.done and getmtime(__file__) != state.file_time:
                restart_requested = True
                print("Restart requested due to file change.")
                
            # Clean up resources on exit
            try:
                # Make sure js_linux's done flag is set before deinitializing
                js.done = True
                time.sleep(0.1)  # Give the thread time to see the done flag
                js.js_deinit()
            except Exception as e:
                print(f"Error during js_deinit: {e}")
                
            try:
                node.destroy_node()
            except Exception as e:
                print(f"Error destroying node: {e}")
                
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as e:
                print(f"Error during ROS shutdown: {e}")
            
            # Restart if requested
            if restart_requested:
                print("Restarting script...")
                time.sleep(0.5)
                # Use os.execv with the correct Python interpreter and arguments
                try:
                    os.execv(sys.executable, [sys.executable] + sys.argv)
                except Exception as e:
                    print(f"Failed to restart: {e}")



# This is the standard Python entry point check.
if __name__ == '__main__':
    main()
