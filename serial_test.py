import serial
import time

s = serial.Serial("/dev/ttyACM1",2000000)
#s.set_low_latency_mode(True)

min = 999999
max = 0
avg = 0
sum = 0
count = 0
last_min = min
last_avg = avg
last_max = max

do_print = False
t_print = 0.0

s.timeout=0.5
result = s.read(530)
result = s.read(530)

time.sleep(2)  # wait for arduino bootloader to finish - arduino resets every time we open a connection 

dt = 0

#tx_data = b'012345678901234567890123456789'
tx_data = b'012345678901234567890123456789012345678901234567890123456789'
#tx_data = b'0123456789'

while True:
  sleep_time = 0.00195 - float(dt/1000000)
  if sleep_time > 0: time.sleep(sleep_time)
  count += 1

  t1=time.monotonic()
  #s.write(b'012345678901234567890123456789012345678901234567890123456789')
  #result = s.read(60)
  #s.write(b'0')
  #s.flush()
  #result = s.read(1)
  s.write(tx_data)
  s.flush()
  rx_data = s.read(len(tx_data))
  
  t2=time.monotonic()

  if rx_data == b'':
    print("read timeout / no data!")
    result = s.read(1000) #get rid of any garbage
    time.sleep(1) #needs to be long enough for the arduino to exit the bootloader
  elif rx_data != tx_data:
    print("rx data != tx data")
    print(rx_data)
    result = s.read(1000) #get rid of any garbage
    time.sleep(1)  #needs to be long enough for the arduino to exit the bootloader

  dt=int((t2-t1)*1000000)

  if dt>max: max = dt
  if dt<min: min = dt
  sum += dt
  avg = int(sum / count)

  if min != last_min: 
    #do_print = True
    last_min=min
  if max != last_max:
    #do_print = True
    last_max=max
  if avg!=last_avg:
    #do_print=True
    last_avg=avg

  if time.monotonic() - t_print > 0.999:
    t_print = time.monotonic()
    do_print = True

  if do_print:
    print("%9.4f   count=%6d   min,avg,max = %5d,%5d,%5d" % (t2,count,min,avg,max ))
    do_print=False
    min = 999999
    max = 0
    avg = 0
    sum = 0
    count = 0
    last_min = min
    last_avg = avg
    last_max = max

