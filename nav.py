#!/usr/bin/env python3

from math import sqrt, atan2, pi
#from turtle import radians

ERR_CIRCLE = 0.013  #2.5cm == 1 inch
DEADZONE = 0.052  #3 degrees  # not used anymore
DOWNRAMP = 0.25
nav_angular_speed = 2.9
nav_linear_speed = 0.3


last_distance = 0.0
debug = True



def rads(degrees):
    """
    convert from degrees to radians
    """
    return degrees * pi / 180.0




def sign(x):
    """
    return -1 if the input was negative, +1 otherwise.
    There probably is a library function for this, but I couldn't find it...
    """
    if x < 0:
        return -1
    else:
        return 1

        


def target_acquired(distance, last_distance):
    """
    returns True if distance falls within the acceptable error
    """
    # if we are inside the error circle, but error is improving, keep going
    # but if we are inside the error circle and we are getting further away, we have reached
    if distance < ERR_CIRCLE and (distance+0.001) > last_distance:
        return True
    else:
        return False




def target_vector(X_position, Y_position, Theta,   X_target, Y_target):
    """
    calculate distance and bearing from target
    returns the tuple (distance,bearing)
    """
    x = X_target - X_position
    y = Y_target - Y_position
    distance = sqrt( (x*x) + (y*y) )
    bearing =  atan2 ( y, x ) - Theta

    #normalize - keep bearing between -pi and +pi
    if bearing > pi:
        bearing = bearing - 2.0*pi
    if bearing < -pi:
        bearing = bearing + 2.0*pi

    return (distance,bearing)




def navigate_target(X_position, Y_position, Theta, X_target,Y_target):
    """
    This is the primary function provided by this module
    produce a navigation command (desired linear + angular velocity) that will take us closer to the target
    also indicate if we reached the target
    """
    global last_distance

    (distance,bearing) = target_vector(X_position, Y_position, Theta, X_target,Y_target)
    acquired = target_acquired(distance, last_distance)
    last_distance = distance

    if acquired:
        return (True, 0.0, 0.0, distance, bearing) # we have reached - return 0.0 for commanded linear and angular velocity
    
    #if ( ( bearing < DEADZONE ) and ( bearing > -DEADZONE ) ) :
    #    commanded_angular_speed = 0.0
    #else:
        #if bearing > DEADZONE:
        #    commanded_angular_speed = nav_angular_speed
        #if bearing < -DEADZONE:
        #    commanded_angular_speed = -nav_angular_speed

    # if bearing is off by >80 degrees, steer hard, otherwise, steer proportionally based on how far off we are
    if abs(bearing) > rads(80):
        commanded_angular_speed = sign(bearing) * nav_angular_speed
        linear_speed_factor = 0.1
    else:
        commanded_angular_speed = (bearing / rads(80)) * nav_angular_speed
        linear_speed_factor = 1.0

    # as we get closer to the target, slow down more and more
    # example: half way between DOWNRAMP and actual target, commanded linear speed will be only 50% of full speed  ( e.g. 0.1 / 0.2 = 0.5 )
    if distance < DOWNRAMP:
        commanded_linear_speed = nav_linear_speed * (distance / DOWNRAMP)
    else:
        commanded_linear_speed = nav_linear_speed

    # apply the speed adjustment factor 
    commanded_linear_speed = commanded_linear_speed * linear_speed_factor


    return (False,commanded_linear_speed, commanded_angular_speed, distance, bearing)




if __name__ == '__main__':

    # test cases

    print( target_vector(0,0,0, 0,0))
    print( target_vector(0,0,0, 1,0))
    print( target_vector(0,0,0, 0,1))
    print( target_vector(0,0,0, 0,-1))

    print( navigate_target(0,0,0, 1,0)  )
 
    print( navigate_target(0,0,0, -1,0)  )
    print( navigate_target(0,0,0, -1,0.1)  )
    print( navigate_target(0,0,0, -1,-0.1)  )

    print( navigate_target(0,0,0, 0,1)  )
    print( navigate_target(0,0,0, 1,1)  )
    print( navigate_target(0,0,0, 1,-1)  )
