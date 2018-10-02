from math import *
from numpy import * as np

# Set the link lengths and starting location
L0 = 8
L1 = 5
L2 = 10
x = 0.2
y = 0.1*x + 10

# Compute IK
G = np.sqrt((x-L0/2.0)*(x-L0/2.0)+y*y)
H = np.sqrt((x+L0/2.0)*(x+L0/2.0)+y*y)

alpha = np.acos((G*G + L0*L0 - H*H)/(2.0*G*L0))
beta = np.acos((H*H + L0*L0 - G*G)/(2.0*H*L0))
gamma = np.acos((G*G + L1*L1 - L2*L2)/(2.0*G*L1))
eta = np.acos((H*H + L1*L1 - L2*L2)/(2.0*H*L1))

th1 = pi - beta - eta
th2 = pi - alpha - gamma

print th1, th2