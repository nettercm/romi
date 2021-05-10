#!/usr/bin/env python3

import time
from time import monotonic as now
from time import sleep
import sys
import os
import posix

print(sys.argv)

interval = float(int(sys.argv[1])) / 1000000.0
offset = float(int(sys.argv[2])) / 1000000.0





def jitter_test(iterations):
  
    for j in range(0,iterations):
      min=99999.0
      max=0.0
      avg=0.0
      sum=0.0
      i=0
      t_print = now()

      while now() - t_print < 1.0:    
        i=i+1
        t1=now()
        time.sleep(interval - offset)
        t2=now()
        t3=t2-t1
        if t3>max: 
            max=t3
        if t3<min:
            min=t3
        sum=sum+t3
    
      avg=sum/i
  
      avg=(avg*1000000)
      min=(min*1000000)
      max=(max*1000000)
      print("min,avg,max,  range = %7.1f , %7.1f , %7.1f ,    %7.1f " % (min,avg,max,max-min))



def loop_test(iterations):
  t_next = float(int(now())+1)
  t1=t_next
  t2=t_next
  print("%9.6f" % (t_next))
  
  min = 9999999.0
  max = 0.0
  i = 0
  for j in range(0,iterations*int(1.0/interval)):
    i = i + 1
    
    #calculate the next time at which we are supposed to do the work; i.e. t_next is a point of time in the future
    #note: this is kept simple on purpose; no attempt to account for numerical issues that will eventually come in
    t_next = t_next + interval
    
    #now calculagte the appropriate amount of sleep time, i.e. "point in time when we want to wake up" - "current time"
    sleep_time = t_next - now()
    
    #now account for the configurable fixed offset, i.e. to deal with the fact that sleep(N) always takes at 
    #least N+offset amount of time
    sleep_time = sleep_time - offset
    
    #in case we missed the bus completely, and we have falled way behind (sleep_time is negative), account for that
    if sleep_time < 0: sleep_time=0
    
    #do the actual sleep()
    sleep( sleep_time  )
    
    #let's see when we actually woke up from sleep
    t1=now()
    
    #we were supposed to wake up at exactly t_next - calculate the error so we can show some stats
    t_error = t1 - t_next
    t_error = t_error*1000000.0   #in microseconds, please....
    
    #eliminate the outlier from the first time through the loop
    if i==1: t_error = 0
    
    if t_error > max: max = t_error
    if t_error < min: min = t_error
    
    #do some useful work here;  
    #the amount of time this workload requires may not be 100% fixed,
    #hence the need for our loop to account for that - which it is....
    
    #show some stats...
    print("%9.6f, %9.6f,  error=%4.0f    min,max,range/jitter=%5.0f,%5.0f,%5.0f" % ( t1, t1-t2 , t_error,min,max,max-min ))
  
    #eliminate the outlier from the first time through the loop
    if i==1: min = 99999.0
  
    t2=t1
  

jitter_test(10)

loop_test(10)