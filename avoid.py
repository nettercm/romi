#!/usr/bin/env python3

from utilities import *

front_min = 0.3     # keep at least this distance away from stuff in front
side_min  = 0.3     # keep at least this distance away from stuff on either side

front_speed = 0.1   # go this slow if there is something right in front
side_speed  = 0.3   # go this slow if there some something on the side (but nothing in front)

turn_speed  = 6.0

def avoid_behavior(r, current_vx, current_vth):
    """
    avoid obstacles
    inputs: range sensor array [12],  current linear speed,   current angular speed
    output: None, if there is no obstacle.    tuple of linear speed , angular speed   otherwise
    """
    front = r[0] #min(r[0],r[1],r[11])    # 90deg cone
    left = r[1] #r[2] #min(r[2],r[3])
    right = r[11] #r[10] #min(r[10],r[9])

    #print("%5.3f  %5.3f  %5.3f      %4.2f %4.2f    %4.2f %4.2f %4.2f    %4.2f %4.2f" % (left,front,right,  r[3],r[2],r[1],r[0],r[11],r[10],r[9]))

    # something right in front?
    if (front < front_min)   or   ( (left < side_min) and (right < side_min) ):  # cover the case of gap rigth in front-center, but obstacle front-left & front-rigth

        # let's slow way down
        linear = slew(current_vx, front_speed, 0.35) 

        # all 3 front-facing sensors < min ?
        if (left < side_min) and (right < side_min):
            print("A   %4.2f,%4.2f,%4.2f   obstacle in front!" % (left,front,right))
            angular = slew(current_vth, sign(current_vth)*turn_speed*2.2, 7.0)  # keep turning in the same direction

        # front + front-left is < min
        elif (left < side_min) and (right >= side_min):
            print("B  %4.2f,%4.2f,%4.2f obstacle in front!" % (left,front,right))
            angular = slew(current_vth, -turn_speed*2.2, 7.0)

        # front + front-rigth is < min
        elif (right < side_min) and (left >= side_min):
            print("C   %4.2f,%4.2f,%4.2f obstacle in front!"  % (left,front,right))
            angular = slew(current_vth, turn_speed*2.2, 7.0)

        else: # OK so only the front is < min, and none of the sides are < min

            if abs(current_vth) > 0.5:  # are we already turning? then keep turning in the same direction
                print("D   %4.2f,%4.2f,%4.2f keep turning"  % (left,front,right))
                angular = slew(current_vth, sign(current_vth)*turn_speed*1.2, 7.0)  # keep turning in the same direction

            elif left <= right:
                print("E   %4.2f,%4.2f,%4.2f obstacle in front!"  % (left,front,right))
                angular = slew(current_vth, -turn_speed*2.2, 7.0)

            else: #elif right < front:
                print("F   %4.2f,%4.2f,%4.2f obstacle in front!"  % (left,front,right))
                angular = slew(current_vth, turn_speed*2.2, 7.0)

        return ( linear , angular )

    elif (left < side_min):
        print("G  %4.2f,%4.2f,%4.2f obstacle on the left!"  % (left,front,right))
        #return ( slew(current_vx, side_speed, 0.15), slew(current_vth, -turn_speed, 7.0) )
        return ( current_vx, slew(current_vth, -turn_speed, 7.0) )
    
    elif (right < side_min):
        print("H  %4.2f,%4.2f,%4.2f obstacle on the right!"  % (left,front,right))
        #return ( slew(current_vx, side_speed, 0.15), slew(current_vth, turn_speed, 7.0) )
        return ( current_vx, slew(current_vth, turn_speed, 7.0) )

    else:
        print("X  %4.2f,%4.2f,%4.2f   "  % (left,front,right))

    # if no obstacle was detected, return nothing
    return None

