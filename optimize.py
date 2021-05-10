import odometry as o
from scanf import scanf


    
    
pattern='%d, %f, %f, %s, %f, %f, %f, %f, %s, %d, %d, %s, %f, %f, %f, %s, %d, %d, %d, %d, %d, %d, %f, %d, %d, %d'



f = open('data.csv')
data = []
print(type(data))
print(len(data))

lines = f.readlines()

for i in range(0,len(lines)):
  data.append(scanf(pattern,lines[i],True))
  
  
print(type(data))
print(len(data))

#print(data[0])

size = len(data)



def calculate():
  l = data[0][ 9]
  r = data[0][10]
  
  o.odometry_update(l,r)
  o.odometry_update(l,r)
  o.x=0.0
  o.y=0.0
  o.th=0.0
  
  for i in range(1,size):
    l = data[i][ 9]
    r = data[i][10]
    o.odometry_update(l,r)


wheeltrack = 0.1400 #0.1415
wheelradius_L = 0.035
wheelradius_R = 0.035

calculate()
baseline_error = (o.x*1000.0)**2.0 + (o.y*1000.0)**2.00 + (o.degs(o.norm(o.th))*100.0)**2
baseline_x = o.x
baseline_y = o.y
baseline_th = o.degs(o.norm(o.th))
print("Baseline: %7.5f,%7.5f,%6.2f  error=%f" % (o.x,o.y,o.degs(o.norm(o.th)), baseline_error) )


for wt in range (1100, 1600, 10):
  wheeltrack = wt / 10000.0
  o.wheeltrack = wheeltrack
  
  for wrl in range (340,360,2):
    wheelradius_L = wrl / 10000.0
    o.wheelradius_L = wheelradius_L
    
    for wrr in range (340,360,2):
      wheelradius_R = wrr / 10000.0
      o.wheelradius_R = wheelradius_R
    
      calculate()
      error = (o.x*1000.0)**2.0 + (o.y*1000.0)**2.00 + (o.degs(o.norm(o.th))*100.0)**2
      th = o.degs(o.norm(o.th))
      
      if (error < baseline_error):
        baseline_error = error
        print("e wt=%7.5f  wrl=%7.5f  wrr=%7.5f   x,y,th=%7.5f,%7.5f,%6.2f  error=%f" % (wheeltrack, wheelradius_L, wheelradius_R, o.x,o.y,o.degs(o.norm(o.th)), error) )
             
      if abs(o.x) < abs(baseline_x):
        baseline_x = o.x
        print("x wt=%7.5f  wrl=%7.5f  wrr=%7.5f   x,y,th=%7.5f,%7.5f,%6.2f  error=%f" % (wheeltrack, wheelradius_L, wheelradius_R, o.x,o.y,o.degs(o.norm(o.th)), error) )
      
      if abs(o.y) < abs(baseline_y):
        baseline_y = o.y
        print("y wt=%7.5f  wrl=%7.5f  wrr=%7.5f   x,y,th=%7.5f,%7.5f,%6.2f  error=%f" % (wheeltrack, wheelradius_L, wheelradius_R, o.x,o.y,o.degs(o.norm(o.th)), error) )
      
      if abs(th) <abs(baseline_th):
        baseline_th = th
        print("t wt=%7.5f  wrl=%7.5f  wrr=%7.5f   x,y,th=%7.5f,%7.5f,%6.2f  error=%f" % (wheeltrack, wheelradius_L, wheelradius_R, o.x,o.y,o.degs(o.norm(o.th)), error) )
  
  
  
  
