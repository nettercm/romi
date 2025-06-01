#!/usr/bin/env python3

# Import necessary modules for device I/O, threading, and system operations
import os, struct, array
from fcntl import ioctl
import time
import threading, queue
import signal
import sys
import select

# We'll store the states here.
axis_states = {} # axis_states: Holds the current value of each axis (e.g., x, y, z, etc.)
button_states = {} # button_states: Holds the current state (pressed/released) of each button
done = False  # Flag to indicate when to stop reading joystick events

# These constants were borrowed from linux/input.h
# Mapping of axis codes to human-readable names
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'throttle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

# Mapping of button codes to human-readable names
button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

# Lists to store the mapping of axes and buttons for the current joystick
axis_map = []
button_map = []
# File handles for the joystick device
jsdev = None
jsdev_os = None
# Thread object for reading joystick events
thread_1=None


def js_thread(q, id):
    """
    Thread function that reads joystick events using a blocking file object.
    
    This function continuously reads joystick events from the jsdev file object,
    unpacks the event data, and updates the corresponding button and axis states.
    It runs until the 'done' flag is set to True or the 'mode' button is pressed.
    
    Args:
        q: A queue object (not currently used but kept for interface consistency)
        id: Thread identifier (not currently used but kept for interface consistency)
    """
    global jsdev, axis_map, axis_states, axis_names, button_map, button_names, button_states, done
    while not done:
        evbuf = jsdev.read(8)  # Read 8 bytes (one event)
        if evbuf:
            t, value, type, number = struct.unpack('IhBB', evbuf)  # Unpack event structure
            #print(struct.unpack('IhBB', evbuf))

            #if type & 0x80:
                #print("(initial)", end="")

            # Button event
            if type & 0x01:
                button = button_map[number]
                if button:
                    button_states[button] = value
                    #if value:
                        #print("%s pressed" % (button))
                    #else:
                        #print("%s released" % (button))

            # Axis event
            if type & 0x02:
                axis = axis_map[number]
                if axis:
                    fvalue = value / 32767.0  # Normalize axis value
                    axis_states[axis] = fvalue
                    #print("%s: %.3f" % (axis, fvalue))

        # If the 'mode' button is pressed, signal to stop
        if button_states['mode'] == 1:
            done = True



def js_thread_v2(q, id):
    """
    Thread function that reads joystick events using a non-blocking file descriptor.
    
    This function continuously checks for joystick events using select() to avoid blocking,
    reads the events when available, unpacks the event data, and updates the corresponding 
    button and axis states. It runs until the 'done' flag is set to True or the 'mode' 
    button is pressed.
    
    Args:
        q: A queue object (not currently used but kept for interface consistency)
        id: Thread identifier (not currently used but kept for interface consistency)
    """
    global jsdev_os, axis_map, axis_states, axis_names, button_map, button_names, button_states, done
    while not done:
        evbuf=None
        # Use select to check if data is available (non-blocking)
        s = select.select([jsdev_os],[],[],0.1)
        if len(s[0]) > 0:
            evbuf = os.read(jsdev_os,8)

        if evbuf:
            t, value, type, number = struct.unpack('IhBB', evbuf)
            #print(struct.unpack('IhBB', evbuf))

            #if type & 0x80:
                #print("(initial)", end="")

            # Button event
            if type & 0x01:
                button = button_map[number]
                if button:
                    button_states[button] = value
                    #if value:
                        #print("%s pressed" % (button))
                    #else:
                        #print("%s released" % (button))

            # Axis event
            if type & 0x02:
                axis = axis_map[number]
                if axis:
                    fvalue = value / 32767.0  # Normalize axis value
                    axis_states[axis] = fvalue
                    #print("%s: %.3f" % (axis, fvalue))

        # If the 'mode' button is pressed, signal to stop
        if button_states['mode'] == 1:
            done = True



def js_init():
    """
    Initialize the joystick device and start the event reading thread.
    
    This function:
    1. Lists available joystick devices
    2. Opens the joystick device (defaults to /dev/input/js0)
    3. Retrieves device information (name, number of axes and buttons)
    4. Sets up axis and button mappings
    5. Starts a non-blocking thread to read joystick events
    
    Returns:
        None
        
    Raises:
        IOError: If the joystick device cannot be opened or accessed
    """
    global fn
    global jsdev
    global jsdev_os
    global thread_1
    # Iterate over the joystick devices.
    print('Available devices:')

    for fn in os.listdir('/dev/input'):
        if fn.startswith('js'):
            print('  /dev/input/%s' % (fn))

    # Open the joystick device (default to js0)
    fn = '/dev/input/js0'

    print('Opening %s...' % fn)
    jsdev = open(fn, 'rb')  # Open as file object (blocking)
    jsdev_os = os.open('/dev/input/js0',os.O_RDONLY|os.O_NONBLOCK)  # Open as file descriptor (non-blocking)

    # Get the device name.
    # The ioctl() call below uses the JSIOCGNAME(len) request code to get the joystick's name string.
    # 0x80006a13 is the base code for JSIOCGNAME, and (0x10000 * len(buf)) encodes the buffer length.
    # The result is stored in 'buf', which is then decoded to a UTF-8 string.
    buf = array.array('B', [0] * 64)
    ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
    js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
    print('Device name: %s' % js_name)

    # Get number of axes and buttons.
    # The following ioctl() calls use JSIOCGAXES and JSIOCGBUTTONS request codes to get the number of axes and buttons.
    # 0x80016a11 (JSIOCGAXES) returns the number of axes supported by the joystick.
    # 0x80016a12 (JSIOCGBUTTONS) returns the number of buttons supported by the joystick.
    buf = array.array('B', [0])
    ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
    num_axes = buf[0]

    buf = array.array('B', [0])
    ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
    num_buttons = buf[0]

    # Get the axis map.
    # The ioctl() call below uses the JSIOCGAXMAP request code (0x80406a32) to get the mapping of axis indices to axis types.
    # The result is stored in 'buf', which contains the axis codes for each axis on the device.
    buf = array.array('B', [0] * 0x40)
    ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

    for axis in buf[:num_axes]:
        axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
        axis_map.append(axis_name)
        axis_states[axis_name] = 0.0

    # Get the button map.
    # The ioctl() call below uses the JSIOCGBTNMAP request code (0x80406a34) to get the mapping of button indices to button types.
    # The result is stored in 'buf', which contains the button codes for each button on the device.
    buf = array.array('H', [0] * 200)
    ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

    for btn in buf[:num_buttons]:
        btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
        button_map.append(btn_name)
        button_states[btn_name] = 0

    print('%d axes found: %s' % (num_axes, ', '.join(axis_map)))
    print('%d buttons found: %s' % (num_buttons, ', '.join(button_map)))

    # Start the joystick event reading thread
    thread_1 = threading.Thread(target=js_thread_v2, args=(0,1))
    thread_1.start()



def js_deinit():
    """
    Deinitialize the joystick device and stop the event reading thread.
    
    This function:
    1. Signals the joystick reading thread to stop by setting the 'done' flag to True
    2. Closes the joystick file object and file descriptor
    3. Waits for the thread to finish execution
    
    Returns:
        None
    """
    global thread_1
    global jsdev
    global jsdev_os
    global done
    done = True  # Signal the thread to stop
    jsdev.close()  # Close the file object
    os.close(jsdev_os)  # Close the file descriptor
    thread_1.join()  # Wait for the thread to finish


# If run as a script, initialize the joystick and print states in a loop
if __name__ == '__main__':
    js_init()
    # Main event loop
    while not done:
        #print("rx: %f" % (axis_states['rx']))
        print(axis_states,button_states)
        time.sleep(0.2)