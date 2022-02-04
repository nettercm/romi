#!/usr/bin/env python3

from math import pi

#sign = lambda x: math.copysign(1, x)



def sign(x):
    """
    return -1 if the input was negative, +1 otherwise.
    There probably is a library function for this, but I couldn't find it...
    """
    if x < 0:
        return -1
    else:
        return 1




def between(x_min,x,x_max):
  if (x >= x_min) and (x <= x_max):
    return True
  else:
    return False
    


def th_delta(th_old, th_new):
  if (abs(th_new) > 170.0) and (sign(th_old) != sign(th_new)):
    if sign(th_new) > 0:
      #clockwise
      return -1.0 * ((180.0 - abs(th_old)) + (180.0 - th_new))
    else:
      return +1.0 * ((180.0 - th_old) + (180.0 - abs(th_new)))
  else:
      return th_new - th_old
      


def rads(degrees):
    """
    convert from degrees to radians
    """
    return degrees * pi / 180.0



def degs(radians):
    """
    convert from radians to degrees
    """
    return radians * 180 / pi



def norm(theta):
    """
    normalize the angle theta so that it always falls between -pi and +pi
    """
    TWO_PI = 2.0 * pi
    normalized = theta % TWO_PI;
    normalized = (normalized + TWO_PI) % TWO_PI;
    if normalized > pi:
        normalized = normalized - TWO_PI;
    return normalized


