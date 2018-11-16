import numpy as np

class TL_Error(Exception):
    def __init__(self, message):
        self.message = message

class TL():
    '''
    This class represents a Two-Link Manipulator. The user may construct a given
    manipulator with the size of the two links. The links are treated as resting
    along the x axes, and thus have a 0 value for theta_1, and a 180 for theta_2.
    
    Parameters
    ----------
    link_1:  Numeric
        This is the size of the second set of links. Each attached directly to the end 
        of the top_bar.
    link_2:   Numeric
        These are the final set of links. These directily attached to link_1 and to 
        one another ending in an effector. 
    '''

    def __init__(self, link_1, link_2):
        self._link_1 = link_1
        self._link_2 = link_2
        self._theta_1 = 0
        self._theta_2 = 0
        self._x = link_1 + link_2
        self._y = 0

    @property
    def link_1(self):
        '''
        The length of the first link in a two link manipulator 
        '''
        return self._link_1

    @link_1.setter
    def link_1(self, value):
        self._link_1 = value

    @property
    def link_2(self):
        '''
        The length of the second link in a two link manipulator 
        '''
        return self._link_2

    @link_2.setter
    def link_2(self, value):
        self._link_2 = value


    @property
    def theta_1(self):
        '''
        The angle represented between the initial connection point and the
        first link.
        '''
        return self._theta_1

    @theta_1.setter
    def theta_1(self, value):
        self._theta_1 = value


    @property
    def theta_2(self):
        '''
        The angle represented between the link 1 and link 2.
        '''
        return self._theta_2

    @theta_2.setter
    def theta_2(self, value):
        self._theta_2 = value


    @property
    def x(self):
        '''
        The position in the horizontal direction of the end effector
        '''

        return self._x

    @x.setter
    def x(self, value):
        self._x = value

    @property
    def y(self):
        '''
        The position in the vertical direction of the end effector
        '''
        return self._y

    @y.setter
    def y(self, value):
        self._y = value

    def FK( self, theta_1,  theta_2 ):
        '''
        This function takes to given angles, theta_1 and theta_2 and returns the position
        of the end effector.

        Parameters
        ----------
        theta_1:       Numeric
            This is a given angle between link_1 and 0
        theta_2      Numeric
            This is a given angle between link_2 and link_1

        Returns
        -------
        pos_list:       list( list(Numeric) length 2) length n
            This is the position that the end effector is located.
            The list is structured as [x, y].
        '''
        pos_list=[]
        self._x = self._link_2*np.cos(theta_1+ttheta_2) + self._link_1*np.cos(theta_1)
        self._y = self._link_2*np.sin(theta_1+ttheta_2) + self._link_1*np.sin(theta_1)
        
        pos_list.append([self._x, self._y])
        return pos_list

    def IK( self, xlist,  ylist ):
        '''
        This function 

        Parameters
        ----------
        xlist:       Numeric
            Gives the x position of the Two-Link Manipulator
        ylist:       Numeric
            Gives the y position of the Two-Link Manipulator

        Returns
        -------
        ang_list:       list( list(Numeric) length 2) length n
            This is 
        '''
        ang_list=[]

        d =  (xlist*xlist+ylist*ylist-self._link_1*self._link_1-self._link_2*self._link_2)/(2*self._link_1*self._link_2)
        self._theta_2 = np.arctan2(-np.sqrt(1.0-d*d),d)
        self._theta_1 = np.arctan2(ylist,xlist) - np.arctan2(self._link_2*np.sin(self._theta_2),self._link_1+self._link_2*np.cos(self._theta_2))
        
        return (self._theta_1, self._theta_2)


