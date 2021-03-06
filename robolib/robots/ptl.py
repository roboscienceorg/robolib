import numpy as np

class PTL_Error(Exception):
    def __init__(self, message):
        self.message = message

class PTL():
    '''
    This class represents a Two-Link Parallel Manipulator. The user may construct a 
    Two-Linked Parallel with sizes for the first link size and second link size.
    '''

    def __init__(self, top_bar, link_1, link_2, starting_pos=(0.0, 0.0)):

        '''
        This function takes in a set of links that can be used to represent a Parallel 
        Two-Link manipulator.

        Parameters
        ----------
        top_bar:  Numeric
            This is the base bar the two first links are attached to. X and Y coordinates
            are based upon the middle of this bar being the (0,0) coordinate.
        link_1:  Numeric
            This is the size of the second set of links. Each attached directly to the end 
            of the top_bar.
        link_2:   Numeric
            These are the final set of links. These directily attached to link_1 and to 
            one another ending in an effector. 

        '''
        self._top_bar = top_bar
        self._link_1 = link_1
        self._link_2 = link_2
        self._x = starting_pos[0]
        self._y = starting_pos[1]

        if (self._link_1 <= 0 and self._link_2 <= 0):
            raise PTL_Error("At least one arm links must be greater than 0")


    @property
    def top_bar(self):
        '''
        T 
        '''
        return self._top_bar

    @top_bar.setter
    def top_bar(self, value):
        self._top_bar = value

    @property
    def link_1(self):
        '''
        T 
        '''

        return self.link_1

    @link_1.setter
    def link_1(self, value):
        self._link_1 = value

    @property
    def link_2(self):
        '''
        T 
        '''
        
        return self.link_2

    @link_2.setter
    def link_2(self, value):
        self._link_2 = value

    @property
    def position(self):
        '''
        T 
        '''

        return [self._x, self._y]

    @position.setter
    def position(self, value):
        self._x = value[0]
        self._y = value[1]

    @property
    def x(self):
        '''
        T 
        '''

        return self._x

    @x.setter
    def x(self, value):
        self._x = value

    @property
    def y(self):
        '''
        T 
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
            This is a given angle between link_1 and link_2 on the left side
        theta_2      Numeric
            This is a given angle between link_1 and link_2 on the right side

        Returns
        -------
        pos_list:       list( list(Numeric) length 2) length n
            This is the position that the end effector is located.
            The list is structured as [x, y].
        '''
        pos_list = []

        a = -self._link_1 * np.cos(theta_1) - self._top_bar/2
        b = self._link_1* np.sin(theta_1)
        c = self._link_1 * np.cos(theta_2) + self._top_bar/2
        d = self._link_1 *sin(theat_2)

        u = np.sqrt(((a-c)*(a-c))+((b-d)*(b-d)))
        v = (self._link_2* self._link_2) - (u*u)/4

        self._x = ((a+c)/2) + ((v * (b - d))/u)
        self._y = ((b+d)/2) + ((v * (c - a))/u)

        pos_list.append([self._x, self._y])

        return pos_list



            
    def IK( self, xlist,  ylist ):
        '''
        This function takes in an x and y position and uses the given bar size to give an
        accurate angle value between theta_1, the angle on the left manipulators, and theta_2,
        the angle on the right maniulators.

        Parameters
        ----------
        xlist:       Numeric
            Gives the x position of the Two-linked Parallel Manipulator
        ylist:       Numeric
            Gives the y position of the Two-Linked Parallel Manipulator

        Returns
        -------
        ang_list:       list( list(Numeric) length 2) length n
            This is the list of angles between link_1 and link_2. It is used to calculate the
            angles bettween the links if only the size of the links are given. The list is 
            structured as [ theta_1, theta_2]
        '''
        ang_list=[]

        G = np.sqrt((xlist-self._top_bar/2.0)*(xlist-self._top_bar/2.0)+ylist*ylist)
        H = np.sqrt((xlist+self._top_bar/2.0)*(xlist+self._top_bar/2.0)+ylist*ylist)

        alpha = np.arccos((G*G + self._top_bar*self._top_bar - H*H)/(2.0*G*self._top_bar))
        beta = np.arccos((H*H + self._top_bar*self._top_bar - G*G)/(2.0*H*self._top_bar))
        gamma = np.arccos((G*G + self._link_1*self._link_1 - self._link_2*self._link_2)/(2.0*G*self._link_1))
        eta = np.arccos((H*H + self._link_1*self._link_1 - self._link_2*self._link_2)/(2.0*H*self._link_1))

        self._theta_1 = np.pi - beta - eta
        self._theta_2 = np.pi - alpha - gamma

        ang_list.append([self._theta_1, self._theta_2])

        return (ang_list)




        '''Add maping funciton?'''