from rnnfilter import *
from math import *
import numpy as np
import pylab as plt
from scipy import linalg
import random


def generate_random_walk(t, positionBounds, maxVel, center_point = None):
    w = np.zeros(len(t))

    for i in range(1, len(t)):
        #Maximum negative position in this timestep to reach position minimum
        minCurVel = positionBounds[0] - w[i-1]
        #Maximum positive position in this timestep to reach position max
        maxCurVel = positionBounds[1] - w[i-1]

        #Min and max motor diff including maxDiff
        minCurVel = max(-maxVel, minCurVel)
        maxCurVel = min(maxVel, maxCurVel)

        temp = random.uniform(minCurVel, maxCurVel)
        if center_point is not None:
            #Apply a random nudge towards the center point
            minRand = min(0, w[i] - center_point)
            maxRand = max(0, w[i] - center_point)
            temp = temp - random.uniform(minRand, maxRand)
            temp = max(minCurVel, min(maxCurVel, temp))
        w[i] = w[i-1] + temp


    return w

def generateWheelData(t, bounds, dd, L):
    speed = generate_random_walk(t, bounds, abs(bounds[1])/100) #forward speed of car
    dTheta = generate_random_walk(t, (-pi/24, pi/24), pi/24, center_point=0) #angle delta of car

    diff = L*dTheta/dd

    w1 = speed + diff
    w2 = speed - diff
            
    return w1, w2

def generateInput(N, mu1, sigma1, mu2, sigma2, minVel, maxVel, dd, L):
    # set up all the input data
    x = np.zeros((N,3))
    z = np.zeros((N,3))
    t = np.linspace(0, 40, N)
    #w1 = 1.5*np.sin(t)
    #w2 = 1.0*np.cos(t)
    w1, w2 = generateWheelData(t, (minVel, maxVel), dd, L)

    k = 1
    while (k<N):
        q = np.random.normal(mu1,sigma1,3) # process error
        r = np.random.normal(mu2,sigma2, 3) # observation error
        # set up x with the process noise
        # x holds the actual position of the robot
        x[k,0] = x[k-1,0] + dd*(w1[k]+w2[k])*cos(x[k-1,2]) + q[0] # x
        x[k,1] = x[k-1,1] + dd*(w1[k]+w2[k])*sin(x[k-1,2]) + q[1] # y
        x[k,2] = x[k-1,2] + dd*(w1[k]-w2[k])/L + q[2] # angle
        # set up the observed data
        # z holds what is sent in
        z[k,0] = x[k,0] + r[0] # x
        z[k,1] = x[k,1] + r[1] # y
        z[k,2] = x[k,2] + r[2] # angle
        k = k+1
    return x, z, w1, w2
    
def kf(N, w1, w2, var1, var2, z,dd, L):
    H = np.array([[1,0,0],[0,1,0],[0,0,1]])
    HT = H.T
    V = np.array([[var1,0,0],[0,var1,0],[0,0,var1]])
    W = np.array([[var2,0,0],[0,var2,0],[0,0,var2]])
    P = np.zeros((N,3,3))
    xf = np.zeros((N,3)) # position after kf
    xp = np.zeros(3)
    sp = np.zeros(3)
    k = 1
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
    return xf

def main():
    N = 400 # length of input
    mu1, sigma1 = 0.0, .01#0.025 # process error
    mu2, sigma2 = 0.0, .2#0.85 # observation error
    var1 = sigma1*sigma1 
    var2 = sigma2*sigma2
    dt = 0.1
    r = 4
    dd = r*dt/2.0
    L = 6
    minVel = .2 
    maxVel = .3



    # x holds the actual position
    # z holds the noisy data

    #init with how accurate the input is?
    # how the array sent in relates - how to build the network
    f = rnnFilter()

    #training
    k = 0
    while k < 40000 :
        print("Train set : ", k)
        #generate input data
        x, z, w1, w2 = generateInput(N, mu1, sigma1, mu2, sigma2, minVel, maxVel, dd, L)
        f.train(z, x)
        #plt.plot(x[:,0], x[:,1], 'b.')
        #plt.savefig(str(k)+".png")
        #plt.close()
        k = k + 1

    fname = "test.h5"
    #test save and load
    f.save(fname)
    f.load(fname)


    # get the average loss
    loss = 0
    k = 0
    while k < 1000 :
        x, z, w1, w2 = generateInput(N, mu1, sigma1, mu2, sigma2, minVel, maxVel, dd, L)
        loss = loss + f.eval(z,x)
        k = k + 1
    print("loss: ", loss/k)


    xf = np.zeros((N,3)) # position after kf
    #generate input data
    x, z, w1, w2 = generateInput(N, mu1, sigma1, mu2, sigma2, minVel, maxVel, dd, L)
    k = 0
    while k < N :
        xf[k] = f.run(z[k])
        k = k+1

    print(f.eval(z,x))
    print(((x - xf)**2).mean(axis=None))
    f2 = kf(N, w1, w2, var1, var2, z, dd, L)

    xf = np.reshape(xf, (xf.shape[0], xf.shape[1]))
    mse = ((x - f2)**2).mean(axis=None)
    print(mse)

    t = np.arange(0,N,1)
    plt.plot(t, x, 'b.', t,z,'r.', t, xf,'g.')
    plt.savefig("a.png")
    plt.close()
    plt.plot(x[:,0], x[:,1], 'b.',z[:,0], z[:,1] ,'r.', xf[:,0], xf[:,1],'g.', f2[:,0], f2[:,1], 'm.')
    plt.savefig("b.png")





if __name__ == '__main__':
    main()
