from math import *

#Arm Lengths
a1,a2 = 15.0,10.0

#Position
x,y = 10.0,8.0
d =  (x*x+y*y-a1*a1-a2*a2)/(2*a1*a2)

#Kinematic Equations
t2 = atan2(-sqrt(1.0-d*d),d)
t1 = atan2(y,x) - atan2(a2*sin(t2),a1+a2*cos(t2))
print t1,t2


x1 = a2*cos(t1+t2) + a1*cos(t1)
y1 = a2*sin(t1+t2) + a1*sin(t1)
print x1, y1
