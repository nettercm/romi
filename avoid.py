#!/usr/bin/env python3

from utilities import *
import time

front_min = 0.3     # keep at least this distance away from stuff in front
side_min  = 0.3     # keep at least this distance away from stuff on either side

front_speed = 0.1   # go this slow if there is something right in front
side_speed  = 0.3   # go this slow if there some something on the side (but nothing in front)

turn_speed  = 6.0

vx_slew_rate = 0.35
vth_slew_rate= 7.0

'''

09  10  11  12  01  02  03
        0   0   0               No obstacle
        0   0   1               right only.     keep vx.  turn left.
        0   1   0               front.          reduce vx.  if turning, keep turning (but turn faster). if not turning, start turning
        0   1   1               front.          reduce vx.  turn left.
        1   0   0               left.           keep vx.   turn right.
        1   0   1               front.          reduce vx.  if turning, keep turning (but turn faster). if not turning, start turning
        1   1   0               front.          reduce vx.  turn right.
        1   1   1               front.          reduce vx.   keep turning.

'''

counter = 0

def avoid_behavior(r, current_vx, current_vth):
    """
    avoid obstacles
    inputs: range sensor array [12],  current linear speed,   current angular speed
    output: None, if there is no obstacle.    tuple of linear speed , angular speed   otherwise
    """
    global counter

    if True:
        front = r[0] 
        left = min(r[1],r[2]) 
        right = min(r[11],r[10]) 
    else:
        front = min(r[0],r[1],r[11])   
        left = min(r[2],r[3])
        right = min(r[10],r[9])


    t = time.monotonic()

    #print("%5.3f  %5.3f  %5.3f      %4.2f %4.2f    %4.2f %4.2f %4.2f    %4.2f %4.2f" % (left,front,right,  r[3],r[2],r[1],r[0],r[11],r[10],r[9]))

    # something right in front?
    if (front < front_min)   or   ( (left < side_min) and (right < side_min) ):  # cover the case of gap rigth in front-center, but obstacle front-left & front-rigth
        counter = 0
        # let's slow way down
        linear = slew(current_vx, front_speed, vx_slew_rate) 

        # all 3 front-facing sensors < min ?
        if (left < side_min) and (right < side_min):
            if abs(current_vth) > 0.5:  # are we already turning? then keep turning in the same direction
                print("A   %4.2f,%4.2f,%4.2f obstacle in front!  keep turning"  % (left,front,right))
                angular = slew(current_vth, sign(current_vth)*turn_speed*2.2, vth_slew_rate)  # keep turning in the same direction

            elif left <= right:
                print("B   %4.2f,%4.2f,%4.2f obstacle in front-left!"  % (left,front,right))
                angular = slew(current_vth, -turn_speed*2.2, vth_slew_rate)

            else: #elif right < front:
                print("C   %4.2f,%4.2f,%4.2f obstacle in front-right!"  % (left,front,right))
                angular = slew(current_vth, turn_speed*2.2, vth_slew_rate)

        # front + front-left is < min
        elif (left < side_min) and (right >= side_min):
            print("D  %4.2f,%4.2f,%4.2f obstacle in front-left!" % (left,front,right))
            angular = slew(current_vth, -turn_speed*2.2, vth_slew_rate)

        # front + front-rigth is < min
        elif (right < side_min) and (left >= side_min):
            print("E   %4.2f,%4.2f,%4.2f obstacle in front-right!"  % (left,front,right))
            angular = slew(current_vth, turn_speed*2.2, vth_slew_rate)

        else: # OK so only the front is < min, and none of the sides are < min

            if abs(current_vth) > 0.5:  # are we already turning? then keep turning in the same direction
                print("F   %4.2f,%4.2f,%4.2f keep turning"  % (left,front,right))
                angular = slew(current_vth, sign(current_vth)*turn_speed*1.2, vth_slew_rate)  # keep turning in the same direction

            elif left <= right:
                print("G   %4.2f,%4.2f,%4.2f obstacle in front!"  % (left,front,right))
                angular = slew(current_vth, -turn_speed*2.2, vth_slew_rate)

            else: #elif right < front:
                print("H   %4.2f,%4.2f,%4.2f obstacle in front!"  % (left,front,right))
                angular = slew(current_vth, turn_speed*2.2, vth_slew_rate)

        return ( linear , angular )

    elif (left < side_min):
        counter = 0
        print("I  %4.2f,%4.2f,%4.2f obstacle on the left!"  % (left,front,right))
        #return ( slew(current_vx, side_speed, 0.15), slew(current_vth, -turn_speed, 7.0) )
        return ( current_vx, slew(current_vth, -turn_speed, vth_slew_rate) )
    
    elif (right < side_min):
        counter = 0
        print("J  %4.2f,%4.2f,%4.2f obstacle on the right!"  % (left,front,right))
        #return ( slew(current_vx, side_speed, 0.15), slew(current_vth, turn_speed, 7.0) )
        return ( current_vx, slew(current_vth, turn_speed, vth_slew_rate) )

    else:
        if counter == 0:
            counter = counter + 1
            print("%8.3f: X  %4.2f,%4.2f,%4.2f   "  % (t,left,front,right))

    # if no obstacle was detected, return nothing
    return None

