import numpy as np
import numbers

from math import sqrt

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
        This is the length of first link attached directly to the base.
    link_2:   Numeric
        This is the length of the second link attached to the end of link_1
    '''

    def __init__(self, link_1, link_2):
        '''
        This function creates and initializes the Two-Link Manipulator Object
        with the given link lengths.

        Parameters
        ----------
        link_1: Numeric
            The length of the first link (the one connected to the base)
        link_2: Numeric
            The length of the second link (the one connected to the first)

        Returns
        -------
        TL object with the given parameters

        Raises
        ------
        ValueError:
            When either link_1 or link_2 is not numeric
        '''

        if (not isinstance(link_1, numbers.Number) or not isinstance(link_2, numbers.Number) 
            or link_1 <= 0 or link_2 <= 0):
            raise ValueError("The link lengths must be Numeric and greater than 0")

        self.link_1 = link_1
        self.link_2 = link_2
        self.theta_1 = 0
        self.theta_2 = 0
        self.x = link_1 + link_2
        self.y = 0

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
        The angle represented between the first link and the second link.
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

    def FK( self, joint_angles ):
        '''
        This function takes to given angles, theta_1 and theta_2 and returns the position
        of the end effector.

        Parameters
        ----------
        joint_angles:   list( tuple(Numeric) length 2 ) length n
            This is a list of joint angles which the effector position will be
            calculated for.

        Returns
        -------
        pos_list:       list( Numeric ) length 2
            This is the position that the end effector is located.
            The list is structured as [x, y].

        Raises
        ------
        ValueError: The passed angles are not valid numeric values
        '''
        pos_list = []
        xpos = 0.0
        ypos = 0.0

        for angle_set in joint_angles:
            if len(angle_set) != 2:
                raise ValueError("Invalid Argument - Each set of angles must be of length 2")

            theta_1 = angle_set[0]
            theta_2 = angle_set[1]

            if not isinstance(theta_1, numbers.Number) or not isinstance(theta_2, numbers.Number):
                raise ValueError("Invalid Argument - Both angles must be numeric values")

            xpos = self.link_1*np.cos(theta_1) + self.link_2*np.cos(theta_1+theta_2)
            ypos = self.link_1*np.sin(theta_1) + self.link_2*np.sin(theta_1+theta_2) 

            pos_list.append( (xpos, ypos) )

        # Update state 
        self.theta_1 = joint_angles[-1][0]
        self.theta_2 = joint_angles[-1][1]
        self.x = xpos
        self.y = ypos

        return pos_list


    def IK( self, effector_positions ):
        '''
        This function takes in a list of x and y positions of the end effector 
        and returns a list of joint angles that place the effector there. The
        function treats this as a "poor man's simulation" meaning that the 
        robot will transition to the last effector position in the list.

        i.e. if the list contained [ (5, 0), (3, 1), (2, 3) ], then the
        robot's state after this call would place the end effector at (2, 3).

        NOTE: This function currently returns only one of the two possible values.

        Parameters
        ----------
        effector_positions:       list( tuple(Numeric) length 2 ) length n
            This is a list of n positions (n can be 1) of the effector
            for which the angular values of the arms should be calculated.

        Returns
        -------
        ang_list:       list( tuple(Numeric) length 2 ) length n
            This list contains the angular values of each arm to achieve the
            corresponding effector position. The state of the robot will
            remain as the last item in the list.
        '''

        ang_list=[]
        theta_1 = 0.0
        theta_2 = 0.0

        for position in effector_positions:
            if len(position) != 2:
                raise ValueError("Invalid Argument - Each position in the list must be of length 2")

            xpos = position[0]
            ypos = position[1]

            if not isinstance(theta_1, numbers.Number) or not isinstance(theta_2, numbers.Number):
                raise ValueError("Invalid Argument - Both x and y values must be numeric")
            elif sqrt(xpos**2+ ypos**2) > (self.link_1 + self.link_2):
                raise ValueError("Invalid Argument - The x and y position specified is beyond the robot's reachable area.")

            d = (xpos**2+ypos**2-self.link_1**2-self.link_2**2)/(2*self.link_1*self.link_2)
            theta_2 = np.arctan2(-np.sqrt(1.0-d**2),d)
            theta_1 = np.arctan2(ypos,xpos) - np.arctan2(self.link_2*np.sin(self.theta_2),self.link_1+self.link_2*np.cos(self.theta_2))
            ang_list.append( (theta_1, theta_2) )

        # Update the state
        self.theta_1 = theta_1
        self.theta_2 = theta_2
        self.x = effector_positions[-1][0]
        self.x = effector_positions[-1][1]

        return ang_list


