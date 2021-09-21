#!/usr/bin/env python3

import zmq
import time

iterations = 500000

data = bytes(1000)

context1 = zmq.Context()
publisher = context1.socket(zmq.PUB)
publisher.bind("ipc://testing1")
publisher.send_multipart([b"A", data]) 

#context2 = zmq.Context()
subscriber = context1.socket(zmq.SUB)
result = subscriber.connect("ipc://testing2")
subscriber.setsockopt(zmq.SUBSCRIBE, b"B")


time.sleep(1.5)

t1 = time.monotonic()
t0 = t1

for i in range(0,iterations):
    t_0 = time.monotonic()
    publisher.send_multipart([b"A", data]) 
    #[address, contents] = subscriber.recv_multipart() 
    if i % 1000 == 0:
        t2 = time.monotonic()
        time_per_recv = (t2-t1) / 1000
        time_per_recv *= 1000000  #in microseconds...        
        print("%8d  %6.1f  %6.0f msg/second" % (i,time_per_recv, (1.0 / time_per_recv)*1000000))
        t1 = t2
    t_1 = time.monotonic()
    sleep_time = (0.001 - (t_1-t_0) - 0.000020)
    if(sleep_time > 0):
        time.sleep(sleep_time)

t2 = time.monotonic()

time_per_pub = (t2-t0) / iterations
time_per_pub *= 1000000  #in microseconds...
print( time_per_pub )
