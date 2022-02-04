#!/usr/bin/env python3

th = 0.3


def avoid_behavior(r):
    front = min(r[0],r[1],r[11])
    left = min(r[2],r[3])
    right = min(r[10],r[9])

    print("%5.3f  %5.3f  %5.3f      %4.2f %4.2f    %4.2f %4.2f %4.2f    %4.2f %4.2f" % (left,front,right,  r[3],r[2],r[1],r[0],r[11],r[10],r[9]))
    
    linear = 0.1

    if (front < th):  #or (left < th) or (right < th):
        if left <= right:
            print("turn right")
            angular = -3.0
        else: #elif right < front:
            print("turn left")
            angular = 3.0

        return (linear, angular)
    
    return None

