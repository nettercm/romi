import sys
import select
import tty
import termios
import time

done = False


def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)
try:
    tty.setcbreak(sys.stdin.fileno())

    i = 0
    while not done:
    
        time.sleep(0.02)
        #print(i)
        #i += 1

        t1 = time.monotonic()
        while isData():
            c = sys.stdin.read(1)
            print(hex(ord(c[0])))
            if c == ' ':         # x1b is ESC
                done = True
                break

        t2 = time.monotonic()
        print("loop time = %7.5f" % (t2-t1) )
        
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    
    
