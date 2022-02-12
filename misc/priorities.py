#!/usr/bin/env python3
import psutil 
import os
import time

kthreadd=None

def get_kthreadd():
    global proc
    for proc in psutil.process_iter():
        try:
            # Get process name & pid from process object.
            #name = proc.name()
            pid = proc.pid
            #ppid = proc.ppid()
            #sched = os.sched_getscheduler(pid)
            #prio = os.sched_getparam(pid).sched_priority
            if pid == 2:
                return

        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass


#get_kthreadd()
kthreadd = psutil.Process(pid=2)

while True:
    time.sleep(5.0)
    # Iterate over all running process
    for proc in kthreadd.children():
        try:
            # Get process name & pid from process object.
            name = proc.name()
            pid = proc.pid
            ppid = proc.ppid()
            sched = os.sched_getscheduler(pid)
            prio = os.sched_getparam(pid).sched_priority
            if ppid == 2:
                if sched == 0:
                    print("time=%11.1f  name=%40s  pid=%6d  sched=%d  prio=%3d  ppid=%6d" % (time.monotonic(),name,pid,sched,prio,ppid))
                    param = os.sched_param(1)
                    os.sched_setscheduler(pid, os.SCHED_RR, param)

        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
