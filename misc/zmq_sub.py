#!/usr/bin/env python3

import zmq
import time

count = 0
t1 = 0
t2 = 0

data = bytes(1000)

context1 = zmq.Context()
subscriber = context1.socket(zmq.SUB)
result = subscriber.connect("ipc://testing1")
print(result)
subscriber.setsockopt(zmq.SUBSCRIBE, b"A")
print(subscriber)

#context2= zmq.Context()
publisher = context1.socket(zmq.PUB)
publisher.bind("ipc://testing2")
print(publisher)
#publisher.send_multipart([b"B", data]) 

time.sleep(0.5)

t1 = time.monotonic()

while True:
    [address, contents] = subscriber.recv_multipart() 
    #publisher.send_multipart([b"B", data]) 
    count += 1
    if count % 1000 == 0:
        t2 = time.monotonic()
        time_per_recv = (t2-t1) / 1000
        time_per_recv *= 1000000  #in microseconds...        
        print("%8d  %6.1f  %6.0f msg/second" % (count,time_per_recv, (1.0 / time_per_recv)*1000000))
        t1 = t2
