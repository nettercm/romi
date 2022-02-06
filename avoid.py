#!/usr/bin/env python3

front_min = 0.3     # keep at least this distance away from stuff in front
side_min  = 0.3     # keep at least this distance away from stuff on either side

front_speed = 0.0   # go this slow if there is something right in front
side_speed  = 0.1   # go this slow if there some something on the side (but nothing in front)

turn_speed  = 3.0

def avoid_behavior(r):
    """
    avoid obstacles
    inputs: current linear speed,   current angular speed,  range sensor array [12]
    output: None, if there is no obstacle.    tuple of linear speed , angular speed   otherwise
    """
    front = r[0] #min(r[0],r[1],r[11])    # 90deg cone
    left = r[1] #r[2] #min(r[2],r[3])
    right = r[11] #r[10] #min(r[10],r[9])

    print("%5.3f  %5.3f  %5.3f      %4.2f %4.2f    %4.2f %4.2f %4.2f    %4.2f %4.2f" % (left,front,right,  r[3],r[2],r[1],r[0],r[11],r[10],r[9]))
    
    if (front < front_min):  #or (left < th) or (right < th):
        print("obstacle in front!")
        if left <= right:
            #print("turn right")
            angular = -turn_speed
        else: #elif right < front:
            #print("turn left")
            angular = turn_speed
        return (front_speed, angular)

    elif (left < side_min):
        print("obstacle on the left!")
        return (side_speed, -turn_speed)
    
    elif (right < side_min):
        print("obstacle on the right!")
        return (side_speed, turn_speed)

    # if no obstacle was detected, return nothing
    return None

