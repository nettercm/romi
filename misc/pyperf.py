import time
import os
import math
import re


x = 1
y = 1.0
string1 = "(this).*(test)"
string2 = "this is a test"

def test():
  global x,y,string1,string2
  for i in range(1,500000):
  	#continue     # ~ 0.4us per iteration
  	x = x + 1     # ~ 0.6us 
  	y = y + 1.0   # ~ 0.45us
  	z = math.sin(y)  # 1.2us
  	w = math.cos(z)  # 1.3us
  	t2=time.monotonic()  # ~1.2us
  	t = time.asctime()   # ~ 8.3us
  	result = re.search(string1,string2)  # ~ 9us
  	
   
t1=time.monotonic()
test()
t2=time.monotonic()
print(((t2-t1)/500000)*1000000)

t1=time.monotonic()
test()
t2=time.monotonic()
print(((t2-t1)/500000)*1000000)



