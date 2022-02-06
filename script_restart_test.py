#!/usr/bin/env python3

import os
import sys
import time
from os.path import getmtime

WATCHED_FILES = [__file__]


WATCHED_FILES_MTIMES = [(f, getmtime(f)) for f in WATCHED_FILES]

print("starting....")

while True:
    for f, mtime in WATCHED_FILES_MTIMES:
        if getmtime(f) != mtime:
            print("restarting....")
            time.sleep(0.5)
            os.execv(__file__, sys.argv)
    time.sleep(0.1)
