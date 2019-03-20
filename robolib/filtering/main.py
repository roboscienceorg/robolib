from rnnfilter import *
from math import *
import numpy as np
import pylab as plt
from scipy import linalg
#loop
#read in the made up input sensor data, or create sensor data
#make the data noisy
#send the data into the neural net
#when training send in the original unnoisy data as well
#print out how the neural net did
def main():

    N = 100 # length of input
    mu1, sigma1 = 0.0, 0.025 # process error
    mu2, sigma2 = 0.0, 0.85 # observation error
    var1 = sigma1*sigma1 
    var2 = sigma2*sigma2
    dt = 0.1
    r = 4
    dd = r*dt/2.0
    L = 6
    x = np.zeros((N,3))
    z = np.zeros((N,3))
    t = np.linspace(0, 10, 100)
    w1 = 1.5*np.sin(t)
    w2 = 1.0*np.cos(t)
    k = 1
    # set up all the input data
    while (k<N):
        q = np.random.normal(mu1,sigma1,3) # process error
        r = np.random.normal(mu2,sigma2, 3) # observation error
        # set up x with the process noise
        # x holds the actual position of the robot
        x[k,0] = x[k-1,0] + dd*(w1[k]+w2[k])*cos(x[k-1,2]) + q[0] # x
        x[k,1] = x[k-1,1] + dd*(w1[k]+w2[k])*sin(x[k-1,2]) + q[1] # y
        x[k,2] = x[k-1,2] + dd*(w1[k]-w2[k])/L + q[2] # ?
        # set up the observed data
        # z holds what is sent in
        z[k,0] = x[k,0] + r[0] # x
        z[k,1] = x[k,1] + r[1] # y
        z[k,2] = x[k,2] + r[2] # ?
        k = k+1

    #H = np.array([[1,0,0],[0,1,0],[0,0,1]])
    #HT = H.T
    #V = np.array([[var1,0,0],[0,var1,0],[0,0,var1]])
    #W = np.array([[var2,0,0],[0,var2,0],[0,0,var2]])
    #P = np.zeros((N,3,3))
    xf = np.zeros((N,3)) # position after kf
    #xp = np.zeros(3)
    #sp = np.zeros(3)
    #k = 1
    """
    while (k<N):
        xp[0] = xf[k-1,0] + dd*(w1[k]+w2[k])*cos(xf[k-1,2])
        xp[1] = xf[k-1,1] + dd*(w1[k]+w2[k])*sin(xf[k-1,2])
        xp[2] = xf[k-1,2] + dd*(w1[k]-w2[k])/L
        F1 = [1.0,0.0, -dd*(w1[k]+w2[k])*sin(xf[k-1,2])]
        F2 =[0,1,dd*(w1[k]+w2[k])*cos(xf[k-1,2])]
        F = np.array([F1,F2,[0,0,1]])
        FT = F.T
        pp = np.dot(F,np.dot(P[k-1],FT)) + V
        y = z[k] - np.dot(H,xp)
        S = np.dot(H,np.dot(pp,HT)) + W
        SI = linalg.inv(S)
        kal = np.dot(pp,np.dot(HT,SI))
        xf[k] = xp + np.dot(kal,y)
        P[k] = pp - np.dot(kal,np.dot(H,pp))
        k = k+1
    """




    # x holds the actual position
    # z holds the noisy data

    #init with how accurate the input is?
    # how the array sent in relates - how to build the network
    f = rnnFilter(z[0])

    #training
    f.train(z, x)

    k = 0
    while k < N :
        xf[k] = f.run(z[k])
        k = k+1

    xf = np.reshape(xf, (xf.shape[0], xf.shape[1]))

    t = np.arange(0,N,1)
    plt.plot(t, x, 'b.', t,z,'r.', t, xf,'g.')
    plt.show()
    plt.plot(x[:,0], x[:,1], 'b.',z[:,0], z[:,1] ,'r.', xf[:,0], xf[:,1],'g.')
    plt.show()





if __name__ == '__main__':
    main()
