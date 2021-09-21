#!/usr/bin/env python3
# http://weifan-tmm.blogspot.kr/2015/07/a-simple-turorial-for-python-c-inter.html
import sysv_ipc
import numpy as np
import struct

BUFF_SIZE = 16

from type_definitions import *
SIZEOF_FLOAT = 8
count = 0

msg_string = "." * 128
msg_string += "\0"

try:
    mq1 = sysv_ipc.MessageQueue(1234, sysv_ipc.IPC_CREAT, max_message_size = 8192)
    mq2 = sysv_ipc.MessageQueue(1235, sysv_ipc.IPC_CREAT, max_message_size = 8192)

    while True:
        message, mtype = mq1.receive()
        count += 1
        '''
        print("*** New message received ***")
        print(f"Raw message: {message}")
        if mtype == TYPE_STRING:
            str_message = message.decode()
            print(f"Interpret as string: {str_message}")
        elif mtype == TYPE_TWODOUBLES:
            two_doubles = struct.unpack("dd", message)
            print(f"Interpret as two doubles: {two_doubles}")
        elif mtype == TYPE_NUMPY:
            numpy_message = np.frombuffer(message, dtype=np.int8)
            print(f"Interpret as numpy: {numpy_message}")
        elif mtype == TYPE_DOUBLEANDNUMPY:
            one_double = struct.unpack("d", message[:SIZEOF_FLOAT])[0]
            numpy_message = np.frombuffer(message[SIZEOF_FLOAT:], dtype=np.int8)
            print(f"Interpret as double and numpy: {one_double}, {numpy_message}")
        '''
        #mq2.send(msg_string, True, type=TYPE_STRING)
        mq2.send(message,True,mtype)
        if count % 5000 == 0:
            print(count)

except sysv_ipc.ExistentialError:
    print("ERROR: message queue creation failed")

