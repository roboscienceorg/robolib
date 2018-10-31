import numpy as np

class TL_Error(Exception):
    def __init__(self, message):
        self.message = message

class TL():
	'''
    This class represents a
    '''
	def __init__(self, link_1, link_2):
		self._link_1 = link_1
		self._link_2 = link_2
		self._theta_1 = 0
		self._theta_2 = 0
        '''
        '''
    @property
    def link_1(self):
        """
        T 
        """
        return self._link_1

    @L.setter
    def link_1(self, value):
        self._link_1 = value

    @property
    def link_2(self):
        """
        T 
        """
        return self._link_2

    @L.setter
    def link_2(self, value):
        self._link_2 = value


    @property
    def theta_1(self):
        """
        T 
        """
        return self._theta_1

    @L.setter
    def theta_1(self, value):
        self._theta_1 = value


    @property
    def theta_2(self):
        """
        T 
        """
        return self._theta_2

    @L.setter
    def theta_2(self, value):
        self._theta_2 = value


	@position.setter
    def position(self, value):
        self._x = 0
        self._y = 0

    @property
    def x(self):
        """
        T 
        """

        return self._x

    @x.setter
    def x(self, value):
        self._x = value

    @property
    def y(self):
        """
        T 
        """

        return self._y

    @y.setter
    def y(self, value):
        self._y = value

    def FK( self, theta_1,  theta_2 ):
        """
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
        """

        self._x = link_2*np.cos(theta_1+ttheta_2) + link_1*np.cos(theta_1)
		self._y = link_2*np.sin(theta_1+ttheta_2) + link_1*np.sin(theta_1)
		
		pos_list.append([self._x, self._y])

		return pos_list

    def IK( self, xlist,  ylist ):
        """
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
        """


        d =  (xlist*xlist+ylist*ylist-link_1*link_1-link_2*link_2)/(2*link_1*link_2)

		#Kinematic Equations
		self._theta_2 = np.atan2(-sqrt(1.0-d*d),d)
		self._theta_1 = np.atan2(y,x) - np.atan2(a2*np.sin(t2),a1+a2*np.cos(t2))

		ang_list.append([self._theta_1, self._theta_2])

		return ang_list


