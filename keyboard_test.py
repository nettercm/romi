from pynput import keyboard
import time

def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

# Collect events until released
#with keyboard.Listener(
#        on_press=on_press,
#        on_release=on_release) as listener:
#    listener.join()

# ...or, in a non-blocking fashion:

"""
while True:
    # The event listener will be running in this block
    with keyboard.Events() as events:
        # Block at most one second
        event = events.get(1.0)
        if event is None:
            print('You did not press a key within one second')
        else:
            print('Received event {}'.format(event))
"""
#"""
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)

listener.start()

while True:
    time.sleep(1)
#"""

