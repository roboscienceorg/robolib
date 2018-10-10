from math import *
import numpy as np

class TL_Error(Exception):
    def __init__(self, message):
        self.message = message

class TwoLink():
	'''
    This class represents a
    '''
	def __init__TwoLink():
		self.a1 = a1
		self.a2 = a2
		self.x = x
		self.y = y
        '''
        '''
	d =  (x*x+y*y-a1*a1-a2*a2)/(2*a1*a2)

	#Kinematic Equations
	t2 = np.atan2(-sqrt(1.0-d*d),d)
	t1 = np.atan2(y,x) - np.atan2(a2*np.sin(t2),a1+a2*np.cos(t2))
	print t1,t2


	x1 = a2*np.cos(t1+t2) + a1*np.cos(t1)
	y1 = a2*np.sin(t1+t2) + a1*np.sin(t1)
	print x1, y1
