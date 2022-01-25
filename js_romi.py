#!/usr/bin/env python3

from a_star import AStar
import time
import signal
import sys
import threading
import math
import termios
import fcntl
import os
import js_linux as js
import numpy as np
import statistics 

#from math import cos,sin,pi
#from pynput import keyboard

done = 0

ir_far_xp = [130, 145, 165, 183, 197, 210, 263, 337, 419, 576, 700, 840]
ir_far_fp = [60, 48, 40, 30, 25, 20, 15, 10, 7.5, 5, 4, 3]

ir_near_xp = [65, 112, 201, 257, 325, 467, 580, 660, 800]
ir_near_fp = [30, 20, 10, 8, 6, 4, 3, 2.5, 2]


def signal_handler(sig, frame):
    global done
    print('You pressed Ctrl+C!')
    done = 1
    time.sleep(0.1)
    js.js_deinit()
    a_star.motors(0, 0)
    sys.exit(0)


def sign(x):
    if x >= 0: return 1
    return -1


print("installing SIGINT handler")
signal.signal(signal.SIGINT, signal_handler)

a_star = AStar()


# while 1:
#  a_star.leds(1,0,0)
#  time.sleep(0.0001)
#  a_star.leds(0,1,0)
#  time.sleep(0.0001)
#  a_star.leds(0,0,1)
#  time.sleep(0.0001)

ts0 = 0.0

t_print = time.time()


js.js_init()

max_speed = 300

t_last_print = time.monotonic()

commanded_linear_velocity = 0.0
commanded_angular_velocity = 0.0

target_angular_velocity = 0.0
target_linear_velocity = 0.0

nearest_front_obstacle = 0.0

avoidance_angular_velocity = 60

last_encoders = a_star.read_encoders()
if a_star.error: last_encoders = a_star.read_encoders()

measured_l_speed_history = [0,0,0,0]
measured_r_speed_history = [0,0,0,0]
measured_speed_history_index = 0

l_speed_adjust = 0
r_speed_adjust = 0

avoid_obstacles = False

t_delta = 0.0
t_last_update = time.monotonic()
time.sleep(0.01)
t_target = time.monotonic()

while done == 0:

    t_update = time.monotonic()
    flags = "-"
    turning = False 

    t_delta = t_update - t_last_update
    t_last_update = t_update
    t_delta_error = t_delta - 0.01

    # get all the required inputs from the romi controller
    analog = a_star.read_analog()
    if a_star.error: analog = a_star.read_analog()
    
    battery_millivolts = a_star.read_battery_millivolts()[0]
    if a_star.error: battery_millivolts = a_star.read_battery_millivolts()[0]
    
    encoders = a_star.read_encoders()
    if a_star.error: encoders = a_star.read_encoders()

    measured_l_speed = encoders[0] - last_encoders[0]
    measured_r_speed = encoders[1] - last_encoders[1]
    last_encoders = encoders
    measured_l_speed_history[measured_speed_history_index] = measured_l_speed
    measured_r_speed_history[measured_speed_history_index] = measured_r_speed
    measured_speed_history_index = (measured_speed_history_index + 1) % 4
    avg_measured_l_speed = statistics.mean(measured_l_speed_history)
    avg_measured_r_speed = statistics.mean(measured_r_speed_history)

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

    # when calculating the nearest front obstacle, also consider the NW and NE facing IR sensors, but de-emphasize them 
    nearest_front_obstacle = min( ir_front, us_left, us_right, ir_left*1.5, ir_right*1.5 )

    nearest_side_obstacle = min(ir_left, ir_right)

    t = time.monotonic()

    y = -js.axis_states['y']    # left  joystick, up/down axis
    rx = js.axis_states['rx']   # right joystick  left/right axis   # use z for gamesir controller

    # convert the joystick values into linear and angular velocity up to "max_speed"
    js_linear_velocity = y * max_speed
    js_angular_velocity = rx * max_speed

    if js.button_states['tl'] == 1:
        js_linear_velocity = 100

    # optionally dampen the joystic command; doesn't add much value, so disable for now;  could make this configurable somehow...
    if False:
        target_angular_velocity = (2*target_angular_velocity + js_angular_velocity) / 3
        target_linear_velocity = (5*target_linear_velocity + js_linear_velocity) / 6
    else:
        target_angular_velocity = js_angular_velocity
        target_linear_velocity = js_linear_velocity

    # set the accelleartion (and decelleration) to the default value - might get adjusted by the logic that comes below
    linear_accelleration = 5 #TODO: distinguish between accelleration and decelleartion

    
    # the faster we go, the stronger we need to steer
    avoidance_angular_velocity = (commanded_linear_velocity / 1.0)  + sign(commanded_linear_velocity) #make sure its not zero

    # are we getting too close to something in front WHILE the user is commanding us to go forward?
    # if the user is not trying to go forward, this logic won't kick in!
    if nearest_front_obstacle < 250 and target_linear_velocity > 5:

        flags = flags + "A"
        # calculate the linear velocity (forward speed) as function of how close we are to stuff
        speed_limit = nearest_front_obstacle / 3

        # clamp the target linear velocity to the calculated speed limit, and also adjust accelleration
        if target_linear_velocity > speed_limit:
            flags = flags + "0" 
            target_linear_velocity = speed_limit
            linear_accelleration = 5

        # given that we are close to something in front, when we are also close to something on one side
        # turn towards the other side, i.e. turn towards the side that is more open by forcing target angular velocity to non-zero
        
        if ir_left  > ir_right + 20:
            turning = True
            flags = flags + "1" 
            target_angular_velocity = -avoidance_angular_velocity
        if ir_right > ir_left  + 20: 
            turning = True
            flags = flags + "2" 
            target_angular_velocity = +avoidance_angular_velocity
        
        if us_left  > us_right + 30: 
            turning = True
            flags = flags + "3" 
            target_angular_velocity = -avoidance_angular_velocity
        if us_right > us_left  + 30: 
            turning = True
            flags = flags + "4" 
            target_angular_velocity = +avoidance_angular_velocity
        
        # if not already turning and things are within the dead band, just pick a side...
        if turning == False:
            if abs( ir_left - ir_right ) < 20: 
                flags = flags + "5" 
                target_angular_velocity = -avoidance_angular_velocity
                turning = True
            
            if abs( us_left - us_right ) < 30: 
                flags = flags + "6" 
                target_angular_velocity = -avoidance_angular_velocity
                turning = True
            

    # regardless of what's going on in front, let's check if we are gettting too close on one side (only if user is commanding forward speed)
    if nearest_side_obstacle < 200 and target_linear_velocity > 5 and abs(target_angular_velocity) < 50:

        flags = flags + "B" 

        # the following logic uses a dead band - there needs to be a 20mm difference in the distance readings 
        if ir_left  > ir_right + 20:  
            target_angular_velocity = -avoidance_angular_velocity
            flags = flags + "1" 
            turning = True
        if ir_right > ir_left  + 20:  
            flags = flags + "2" 
            target_angular_velocity = +avoidance_angular_velocity
            turning = True

        # if things are within the dead band, just pick a side...
        if abs( ir_left - ir_right ) < 20: 
            flags = flags + "3" 
            target_angular_velocity = -avoidance_angular_velocity
            turning = True

    if nearest_front_obstacle < 50:
        #target_linear_velocity = 0
        target_angular_velocity = target_angular_velocity * 2

    #if we are turngin to avoid an obstacle, maintain a certain minimum turn speed otherwise there is no point
    if turning:
        if abs(target_angular_velocity) < 15:
            if target_angular_velocity < 0:
                target_angular_velocity = -15
            if target_angular_velocity > 0:
                target_angular_velocity =  15

    if avoid_obstacles == False:
        #for testing purposes, ignore the obstacle avoidance and just listen to the user
        linear_accelleration = 4
        target_angular_velocity = js_angular_velocity
        target_linear_velocity = js_linear_velocity
    

    # ramp up or down to our target angular velocity, using a small deadband of +/- 6 
    if commanded_angular_velocity < target_angular_velocity:              commanded_angular_velocity = commanded_angular_velocity + 5
    if commanded_angular_velocity > target_angular_velocity:              commanded_angular_velocity = commanded_angular_velocity - 5
    if abs( commanded_angular_velocity - target_angular_velocity ) < 6:   commanded_angular_velocity = target_angular_velocity


    # ramp up or down to our target linear velocity, using a small deadband of +/- 6 
    # this uses an accelleration value that might have been adjusted by the obstacle avoidance logic
    if commanded_linear_velocity < target_linear_velocity:                commanded_linear_velocity = commanded_linear_velocity + linear_accelleration
    if commanded_linear_velocity > target_linear_velocity:                commanded_linear_velocity = commanded_linear_velocity - linear_accelleration
    if abs( commanded_linear_velocity - target_linear_velocity ) < 6:     commanded_linear_velocity = target_linear_velocity

    # convert into low-level motor command values for the left and right motor, respectively - for now, same units used here
    # note:  15 encoder ticks per 10ms <=> 100
    avg_measured_l_speed = avg_measured_l_speed * 6.66
    avg_measured_r_speed = avg_measured_r_speed * 6.66

    l_speed = commanded_linear_velocity
    r_speed = commanded_linear_velocity
    l_speed = l_speed + commanded_angular_velocity
    r_speed = r_speed - commanded_angular_velocity

    
    error_l = avg_measured_l_speed - l_speed
    error_r = avg_measured_r_speed - r_speed

    l_speed_adjust = error_l * -3
    r_speed_adjust = error_r * -3

    # the left motor is a little faster than the right motor....
    # TODO: use feedback
    #l_speed = l_speed * 0.92

    #l_speed = l_speed + l_speed_adjust
    #r_speed = r_speed + r_speed_adjust

    # clamp the left/right motor speed values to a certain upper value
    if l_speed >  max_speed: l_speed =  max_speed
    if l_speed < -max_speed: l_speed = -max_speed
    if r_speed >  max_speed: r_speed =  max_speed
    if r_speed < -max_speed: r_speed = -max_speed

    # send the low-level motor command to the romi controller
    a_star.motors(int(l_speed), int(r_speed))
    if a_star.error: a_star.motors(int(l_speed), int(r_speed))

    # once in a while, but not too often, show what's going on
    if t > t_last_print + 0:  #0.05:
        #if abs(t_delta_error) > 0.0002:
        #print("T=%8.4f    %-4.2f  %-4.2f   ir(adc)= %4d,  %4d,  %4d   ir = %5.0f,  %5.0f,  %5.0f   us= %4d,  %4d      %2.1f V      e=%6d,%6d    errors=%d" % (t, js.axis_states['y'],js.axis_states['rx'],    analog[3]/1,  analog[1]/1,  analog[2]/1,   ir_left, ir_front, ir_right,  analog[0]/1,  analog[4]/1,float(battery_millivolts[0])/1000.0, encoders[0], encoders[1], a_star.errors) )
        #print("%-4.2f  %-4.2f" % (js.axis_states['y'],js.axis_states['ry']) )
        print("%8.3f,  %9.6f,  %-6.1f, %-6.1f, %-6.1f, %-6.1f, %-6.1f   ir = %5.0f,  %5.0f,  %5.0f   us= %4d,  %4d   e = %5.1f , %5.1f  m = %-3d,%-3d   v=%3.1f    %s" % 
            (
                time.monotonic(), 
                t_delta_error,
                js_linear_velocity,
                commanded_linear_velocity, 
                commanded_angular_velocity, 
                nearest_front_obstacle, 
                nearest_side_obstacle, 
                ir_left, ir_front, ir_right, 
                us_left, us_right,
                avg_measured_l_speed, avg_measured_r_speed, 
                int(l_speed), int(r_speed), 
                battery_millivolts / 1000.0,
                flags
            ))
        t_last_print = t

    # run the loop at approximately 100Hz
    t_now = time.monotonic()
    t_target = t_target + 0.01
    t_sleep = t_target - t_now
    t_sleep = t_sleep - 0.000025
    if t_sleep > 0:
        time.sleep(t_sleep)

    if js.done == True:
        done = 1


a_star.motors(0, 0)
