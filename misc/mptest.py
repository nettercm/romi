#!/usr/bin/env python3
 
import multiprocessing as mp
import time
import threading, queue
import signal
import sys

"""
results:

muliprocessing latency is 10x compared to threads cpu usage (for a given fixed rate of transactions) is about 4x compared to threads



"""


def signal_handler(sig, frame):
    global done
    print('test.py: You pressed Ctrl+C!')
    done = True
    sys.exit(0)


def main_signal_handler(sig, frame):
    global done
    print('test.py::main(): You pressed Ctrl+C!')
    done = True
    sys.exit(0)




def foo(q,id):
    #q.put('hello')
    print("foo(%d) has started" % (id) )
    t1=time.monotonic()
    for i in range(1,2000):
        time.sleep(id * 0.02)
        t2=time.monotonic()
        print( "%7.4f foo()%1d" % (t2-t1,id) )
        t1=t2


def bar(q,id):
    print("bar(%d) has started" % (id) )
    for i in range(1,2000):
        t1=time.monotonic()
        done=False
        while not done:
            if time.monotonic() > t1 + 0.02:
                done=True


def ping(q1,q2,id):
    #signal.signal(signal.SIGINT, signal_handler)
    print("ping(%d) has started" % (id) )
    
    #first measure latency
    t1=time.monotonic()
    for i in range(1,100000):
        q1.put('ping')
        e = q2.get()
        #print('ping',i,e)
    t2=time.monotonic()
    print("ping: done with overhead test")
    print(i, ((t2-t1)/i)*1000000)
    time.sleep(0.5)

    #now measure cpu usage for doing put() / get() at a fixed rate of 100 per second
    t1=time.monotonic()
    i=0
    while time.monotonic() - t1 < 10.0:
        t = time.monotonic()
        q1.put('ping')
        e = q2.get()
        i = i+1
        #print('ping',i,e)
        sleep_time = 0.005 - (time.monotonic() - t)  #200 iter per sec 
        sleep_time = sleep_time - 0.00010
        if sleep_time > 0: time.sleep(sleep_time)
    print("ping: done with CPU usage test")
    q1.put('done')
    t2=time.monotonic()
    print(i, ((t2-t1)/i)*1000000)
    time.sleep(0.5)


def echo(q1,q2,id):
    #signal.signal(signal.SIGINT, signal_handler)
    print("echo(%d) has started" % (id) )
    t1=time.monotonic()
    done = False
    i = 0
    while not done:
        p = q1.get()
        i = i + 1
        #print('echo',i,p)
        q2.put('echo')
        #if p == 'ping':
        #    q.put('echo')
        if p == 'done':
            done = True
            print("echo(): received the 'done' signal")
 
 
def multiprocessing_test():
    print("multiprocessing test")
    mp.set_start_method('spawn')
    q1 = mp.Queue()
    q2 = mp.Queue()
    print("starting p1")
    p1 = mp.Process(target=ping, args=(q1,q2,1))
    p1.start()
    print("starting p2")
    p2 = mp.Process(target=echo, args=(q1,q2,2))
    p2.start()
    #print(q.get())
    print("waiting for processes to exit...")
    p1.join()
    p2.join()
    print("all sub-processes have exited")
                 
                 

def threading_test():
    print("threading test")
    q1=queue.Queue()
    q2=queue.Queue()
    print("starting t1")
    t1 = threading.Thread(target=ping, args=(q1,q2,1))
    t1.start()
    print("starting t2")
    t2 = threading.Thread(target=echo, args=(q1,q2,2))
    t2.start()
    print("waiting for threads to exit...")
    t1.join()
    t2.join()
    print("all threads have exited")
    
    
    
if __name__ == '__main__':
    print("installing SIGINT handler")
    signal.signal(signal.SIGINT, main_signal_handler)
    time.sleep(1)
    threading_test();
    time.sleep(1)
    multiprocessing_test();
