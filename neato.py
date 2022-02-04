import serial
import time
import scanf
import signal
import numpy
import sys
import threading
import math
import termios
import fcntl
import os
import js_linux as js
import statistics 

from utilities import *

# the following are just for documentation purposes
get_analog_sensors_response  = b'getanalogsensors\r\nSensorName,Unit,Value\r\nBatteryVoltage,mV,13050,\r\nBatteryCurrent,mA,-92,\r\nBatteryTemperature,mC,26297,\r\nExternalVoltage,mV,687,\r\nAccelerometerX,mG,33,\r\nAccelerometerY,mG,25,\r\nAccelerometerZ,mG,961,\r\nVacuumCurrent,mA,0,\r\nSideBrushCurrent,mA,0,\r\nMagSensorLeft,VAL,0,\r\nMagSensorRight,VAL,0,\r\nWallSensor,mm,70,\r\nDropSensorLeft,mm,0,\r\nDropSensorRight,mm,0,\r\n\x1a'
get_digital_sensors_response = b'getdigitalsensors\r\nDigital Sensor Name, Value\r\nSNSR_DC_JACK_IS_IN,0\r\nSNSR_DUSTBIN_IS_IN,0\r\nSNSR_LEFT_WHEEL_EXTENDED,1\r\nSNSR_RIGHT_WHEEL_EXTENDED,1\r\nLSIDEBIT,0\r\nLFRONTBIT,0\r\nLLDSBIT,0\r\nRSIDEBIT,0\r\nRFRONTBIT,0\r\nRLDSBIT,0\r\n\x1a'
get_motors_response          = b'getmotors\r\nParameter,Value\r\nBrush_RPM,0\r\nBrush_mA,0\r\nVacuum_RPM,0\r\nVacuum_mA,0\r\nLeftWheel_RPM,0\r\nLeftWheel_Load%,0\r\nLeftWheel_PositionInMM,0\r\nLeftWheel_Speed,0\r\nRightWheel_RPM,0\r\nRightWheel_Load%,0\r\nRightWheel_PositionInMM,0\r\nRightWheel_Speed,0\r\nSideBrush_mA,0\r\n\x1a'

get_analog_sensors_format  = "getanalogsensors\r\nSensorName,Unit,Value\r\n%s,%s,%d,\r\n%s,%s,%d,\r\n%s,%s,%d,\r\n%s,%s,%d,\r\n%s,%s,%d,\r\n%s,%s,%d,\r\n%s,%s,%d,\r\n%s,%s,%d,\r\n%s,%s,%d,\r\n%s,%s,%d,\r\n%s,%s,%d,\r\n%s,%s,%d,\r\n%s,%s,%d,\r\n%s,%s,%d,\r\n"
get_digital_sensors_format = "getdigitalsensors\r\nDigital Sensor Name, Value\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n"
get_motors_format          = "getmotors\r\nParameter,Value\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n%s,%d\r\n"

port = serial.Serial('/dev/ttyACM0',1000000,timeout=0.005)

running = True
done = False

js.js_init()



#send a command (terminated via \n) and wait for the response (which is terminated with 0x1a)
def send_command_and_get_response(cmd):
    global port
    global done
    port.write(cmd) # send the command string
    response = b''
    while (done == False): # prevent getting stuck here if we are trying to exit    
        response = response + port.read(1000) # just keep appending ...
        if len(response) > 0: 
            if response[len(response)-1] ==26: # ... until we got the "end of response" marker
                break
    return response


# get motor data, e.g. current speed and distrance travelled so far
def get_motors():
    response = send_command_and_get_response(b'getmotors\n')
    s = response.decode('utf-8')
    v = scanf.scanf(get_motors_format,s)
    #print(v)
    return v


# control the motors;  l = left wheel distance,   r = right wheel distance   s = speed
def set_motors(l_dist,r_dist,speed):
    if l_dist == 0: l_dist = 1 
    if r_dist == 0: r_dist = 1 

    if speed == 0:
        cmd = 'setmotor lwheeldisable rwheeldisable\n'
        response = send_command_and_get_response(cmd.encode('utf-8'))
        cmd = 'setmotor lwheelenable rwheelenable\n'
        response = send_command_and_get_response(cmd.encode('utf-8'))
    else:
        cmd = 'setmotor ' + 'lwheeldist ' + str(l_dist) + ' rwheeldist ' + str(r_dist) + ' speed ' + str(speed) + '\n'
        response = send_command_and_get_response(cmd.encode('utf-8'))
    #print(cmd)
    #print(response)



def get_analog_sensors():
    response = send_command_and_get_response(b'getanalogsensors\n')
    s = response.decode('utf-8')
    v = scanf.scanf(get_analog_sensors_format,s)
    #print(v)
    return v



def get_digital_sensors():
    response = send_command_and_get_response(b'getdigitalsensors\n')
    s = response.decode('utf-8')
    v = scanf.scanf(get_digital_sensors_format,s)
    #print(v)
    return v



def test_mode_on():
    response = send_command_and_get_response(b'testmode on\n')
    #print(response)



def signal_handler(sig, frame):
    global running
    global done
    print('You pressed Ctrl+C!')
    #set_motors(0,0)
    running = False
    done = True



def process_laser_scan(data):
    result = [99999,99999,99999,99999,99999,99999,99999,99999,99999,99999,99999,99999,0,0]  # last 2 values are rpm and number of valid readings
    scan = data.decode('utf-8').split('\r\n')
    valid_readings = 0
    for i in range(0,360):
        reading = scanf.scanf('%d,%d,%d,%d',scan[i+2])
        j = i + 10 #15
        if j <    0: j = j + 360
        if j >= 360: j = j - 360
        bucket = int(j / 30)
        mm = reading[1]
        if mm > 32768: mm = mm - 32768 # remove one of the error flags, if present
        if mm > 16384: mm = mm - 16384 # remove the other error flag, if present
        if mm > 0: valid_readings = valid_readings + 1
        # if we have a valid reading, and if its closer than any previous reading, update the value for this bucket
        if ( mm > 0 ) and ( mm < result[bucket] ):   result[bucket] =  mm

    rpm = scanf.scanf('ROTATION_SPEED,%f',scan[362])
    rpm = int(rpm[0]*100)
    result[13] = valid_readings
    result[12] = rpm
    return result



print("installing SIGINT handler")
signal.signal(signal.SIGINT, signal_handler)

response = send_command_and_get_response(b'testmode on\n')
response = send_command_and_get_response(b'setldsrotation on\n')
#set_motors(100,100,50)


def test():
    global done
    global running
    global port

    motor_l_speed = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    motor_r_speed = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    index = 0

    max_speed = 350

    dist = 100

    i = 0

    digital = get_digital_sensors()
    analog = get_analog_sensors()
    motors = get_motors()

    t_lidar = time.monotonic()

    while done == False:

        if js.done == True:
            done = True

        y = -js.axis_states['y']    # left  joystick, up/down axis
        rx = js.axis_states['rx']   # right joystick  left/right axis

        # convert the joystick values into linear and angular velocity up to "max_speed"
        js_linear_velocity = y * max_speed
        js_angular_velocity = rx * max_speed

        l_speed = js_linear_velocity
        r_speed = js_linear_velocity
        l_speed = l_speed + js_angular_velocity
        r_speed = r_speed - js_angular_velocity

        l_speed = int(l_speed)
        r_speed = int(r_speed)

        if abs(l_speed) > abs(r_speed):
            speed  = abs(l_speed)
            l_dist = dist * sign(l_speed)
            r_dist = dist * sign(r_speed)
            try:
                r_dist = abs( (r_speed / l_speed)*dist ) * sign(r_speed)
            except:
                r_dist = dist * sign(r_speed)

        if abs(l_speed) < abs(r_speed):
            speed  = abs(r_speed)
            r_dist = dist * sign(r_speed)
            try:
                l_dist = abs( (l_speed / r_speed)*dist ) * sign(l_speed)
            except:
                l_dist = dist * sign(l_speed)

        if abs(l_speed) == abs(r_speed):
            speed  = abs(r_speed)
            l_dist = dist * sign(l_speed)
            r_dist = dist * sign(r_speed)

        l_dist = int(l_dist)
        r_dist = int(r_dist)
        speed  = int(speed)

        if speed > max_speed:     speed = max_speed
        if abs(l_dist) > 10000:   l_dist = 10000 * sign(l_dist)
        if abs(r_dist) > 10000:   r_dist = 10000 * sign(r_dist)

        try:

            t_now = time.monotonic()
            if t_now > t_lidar + 0.2:
                response = send_command_and_get_response(b'getldsscan\n')
                process_laser_scan(response)
                #print(t_now)
                t_lidar = t_now
                result = process_laser_scan(response)
                front = [0,0,0,0,0,0,0]
                front[0] = result[3]
                front[1] = result[2]
                front[2] = result[1]
                front[3] = result[0]
                front[4] = result[11]
                front[5] = result[10]
                front[6] = result[9]
                print(analog[2],analog[5],analog[38],analog[41],front,result)

            analog = get_analog_sensors()

            drop_sensor_left  = analog[38]
            drop_sensor_right = analog[41]

            if (drop_sensor_left > 1) or (drop_sensor_right > 1):
                l_dist = 0
                r_dist = 0
                speed = 0

            set_motors(l_dist,r_dist,speed)

            if i == 0 : digital = get_digital_sensors()
            if i == 1 : motors = get_motors()
            i = i + 1
            if i > 1: i = 0

            motor_l_speed[index] = motors[15]
            motor_r_speed[index] = motors[23]
            motor_l_speed_avg = numpy.mean(motor_l_speed)
            motor_r_speed_avg = numpy.mean(motor_r_speed)
            index = index + 1
            if index > 19: index = 0

            #print(time.monotonic(),l_speed,r_speed,l1,r1,l2,r2,s,analog[2],analog[5],motors[9],motors[11],motors[13],motors[15],motor_l_speed_avg, motors[17],motors[19],motors[21],motors[23],motor_r_speed_avg)

        except:
            print("either something went wrong or we are exiting...")


    set_motors(0,0,0)
    response = send_command_and_get_response(b'setldsrotation off\n')


if __name__ == '__main__':
    test()


# 100,100,30  / 100,30,30   =>   30, 18
# 100,100,30  / 100,15,30   =>   30, 15 
# 100,100,60  / 100,15,60   =>   60, 32

# 100,100,250 / 100,40,250  =>  248,171
# 100,100,250 / 100,35,250  =>  248,160
# 100,100,250 / 100,25,250  =>  248,153
# 100,100,250 / 100,20,250  =>  248,146
# 100,100,250 / 100,15,250  =>  248,134
# 100,100,250 / 100,10,250  =>  248,128
# 100,100,250 / 100,5, 250  =>  249, 68

# 100,100,150 / 100,20,150  =>  150,88
# 100,100,150 / 100,15,150  =>  152,85
# 100,100,150 / 100,10,150  =>  149,80
# 100,100,150 / 100, 5,150  =>  148,71
# 100,100,150 / 100, 2,150  =>  149,34
