import numpy as np

class MECANUM_Error(Exception):
    def __init__(self, message):
        self.message = message

class MECANUM():
    '''
    This class represents a Mecanum robot. The user may construct
    the object with given 
    Assumes y = 45 degrees
    '''

    def __init__(self, rad, len_front, len_back, starting_pos=(0.0, 0.0, 0.0)):
        '''
        This function creates and initializes the Mecanum with the given parameters.

        Parameters
        ----------
        rad:  Numeric
            The wheel radius
        len_front:  Numeric
            T
        len_back:   Numeric
            Th
        starting_pos: list of 3 Numeric values
            This represents the [x, y, theta] of the robot's initial orientation. This
            value will default to [0, 0, 0]

        Returns
        -------
        DDR object with the given parameters
        '''
        self._rad = rad
        self._len_front = len_front
        self._len_back = len_back
        self._x = starting_pos[0]
        self._y = starting_pos[1]
        self._theta = starting_pos[2]

        if (self._rad <= 0 ):
            raise MECANUM_Error("Wheel Radius must be greater than 0")

        if (self._len_front <= 0 ):
            raise MECANUM_Error("Distance between left and right wheels must be greater than 0")

        if (self._len_back <= 0 ):
            raise MECANUM_Error("Distance between front and back wheels must be greater than 0")

    @property
    def rad(self):
        '''
        T
        '''
        return self._rad

    @rad.setter
    def rad(self, value):
        self._rad = value

    @property
    def len_front(self):
        '''
        T
        '''

        return self._len_front

    @len_front.setter
    def len_front(self, value):
        self._len_front = value

    @property
    def len_back(self):
        '''
        T
        '''

        return self._len_back

    @len_back.setter
    def len_back(self, value):
        self._len_back = value

    @property
    def position(self):
        '''
        The current position of the robot. This can be set as follows: DDR.position = <new_value> or
        individually with x, y, and theta properties.
        '''

        return [self._x, self._y, self._theta]

    @position.setter
    def position(self, value):
        self._x = value[0]
        self._y = value[1]
        self._theta = value[2]

    @property
    def x(self):
        '''
        The x position of the robot. This can be set as follows: DDR.x = <new_value>
        '''

        return self._x

    @x.setter
    def x(self, value):
        self._x = value

    @property
    def y(self):
        '''
        The y position of the robot. This can be set as follows: DDR.y = <new_value>
        '''

        return self._y

    @y.setter
    def y(self, value):
        self._y = value

    @property
    def theta(self):
        '''
        The angular orientation of the robot. This can be set as follows: DDR.theta = <new_value>
        '''

        return self._theta

    @theta.setter
    def theta(self, value):
        self._theta = value


        '''come back and fix how global radians are computed'''
    def global_FK( self, FL, FR, BL, BR, global_radians):
        '''
        This function takes in four rotational velocities and uses them to compute the global
        x, y, and angular velocities of the robot.

        Parameters
        ----------
        FL:      Numeric
            This is the angular velocity, in radians, of the front left wheel
        FR:      Numeric
            This is the angular velocity, in radians, of the front right wheel
        BL:      Numeric
            This is the angular velocity, in radians, of the back left wheel
        BR:      Numeric
            This is the angular velocity, in radians, of the back right wheel

        Returns
        -------
        x_val:      Numeric
            This is the velocity in the x-direction in the global scope

        y_val:      Numeric
            This is the velocity in the y-direction in the global scope

        theta_val: Numeric
            This is the velocity in the angular-direction in the global scope
        '''
        x_vel = global_radians * (self._rad / 4.0) * (FL + FR + BL + BR)
        y_vel = global_radians * (self._rad / 4.0) * (-FL + FR + BL - BR)
        theta_val = global_radians * (self._rad / 4.0) * ((-1/(self._len_back+self._len_front))*(FL) + (1/(self._len_back+self._len_front))*FR + (-1/(self._len_back+self._len_front))*(BL) + (1/(self._len_back+self._len_front))*(BR))


        pos_list.append([x_vel, y_vel, theta_val])

        return pos_list
        #return x_vel, y_vel, theta_val


    def FK( self, FL, FR, BL, BR):
        '''
        This function takes in four rotational velocities and uses them to compute the robotics
        x, y, and angular velocities of the robot.

        Parameters
        ----------
        FL:      Numeric
            This is the angular velocity, in radians, of the front left wheel
        FR:      Numeric
            This is the angular velocity, in radians, of the front right wheel
        BL:      Numeric
            This is the angular velocity, in radians, of the back left wheel
        BR:      Numeric
            This is the angular velocity, in radians, of the back right wheel

        Returns
        -------
        x_vel:      Numeric
            This is the velocity in the x-direction in the global scope

        y_vel:      Numeric
            This is the velocity in the y-direction in the global scope

        theta_val: Numeric
            This is the velocity in the angular-direction in the global scope
        '''
        x_vel = (self._rad / 4.0) * (FL + FR + BL + BR)
        y_vel = (self._rad / 4.0) * (-FL + FR + BL - BR)
        theta_val = (self._rad / 4.0) * ((-1/(self._len_back+self._len_front))*(FL) + (1/(self._len_back+self._len_front))*FR + (-1/(self._len_back+self._len_front))*(BL) + (1/(self._len_back+self._len_front))*(BR))

        return x_vel, y_vel, theta_val


    def IK( self, x_val, y_val, theta_val):
        '''
        This function takes in a list of angular velocities and computes
        the positions of the robot and returns this individual wheel velocities.

        Parameters
        ----------
        x_val:      Numeric
            This is the velocity in the x-direction in the local scope

        y_val:      Numeric
            This is the velocity in the y-direction in the local scope

        theta_val: Numeric
            This is the velocity in the angular-direction in the local scope

        Returns
        -------
        FL:      Numeric
            This is the angular velocity, in radians, of the front left wheel
        FR:      Numeric
            This is the angular velocity, in radians, of the front right wheel
        BL:      Numeric
            This is the angular velocity, in radians, of the back left wheel
        BR:      Numeric
            This is the angular velocity, in radians, of the back right wheel
        '''

        FL = (1/self._rad) * (x_val - y_val - theta_val*(self._len_back+self._len_front))
        FR = (1/self._rad) * (x_val + y_val + theta_val*(self._len_back+self._len_front))
        BL = (1/self._rad) * (x_val + y_val - theta_val*(self._len_back+self._len_front))
        BR = (1/self._rad) * (x_val - y_val + theta_val*(self._len_back+self._len_front))

        return FL, FR, BL, BR


    def global_IK( self, x_val, y_val, theta_val):
        '''
        This function takes in a list of angular velocities globally and computes
        the positions of the robot and returns this individual wheel velocities.

        Parameters
        ----------
        x_val:      Numeric
            This is the velocity in the x-direction in the global scope

        y_val:      Numeric
            This is the velocity in the y-direction in the global scope

        theta_val: Numeric
            This is the velocity in the angular-direction in the global scope

        Returns
        -------
        FL:      Numeric
            This is the angular velocity, in radians, of the front left wheel
        FR:      Numeric
            This is the angular velocity, in radians, of the front right wheel
        BL:      Numeric
            This is the angular velocity, in radians, of the back left wheel
        BR:      Numeric
            This is the angular velocity, in radians, of the back right wheel
        '''

        x = (np.cos(theta_val)*x_val + np.sin(theta_val)*y_val)
        y = (-np.sin(theta_val)*x_val + np.cos(theta_val)*y_val)

        FL = (1/self._rad) * (x - y - theta_val*(self._len_back+self._len_front))
        FR = (1/self._rad) * (x + y + theta_val*(self._len_back+self._len_front))
        BL = (1/self._rad) * (x + y - theta_val*(self._len_back+self._len_front))
        BR = (1/self._rad) * (x - y + theta_val*(self._len_back+self._len_front))

        return FL, FR, BL, BR



    def __str__(self):
        '''Provides a human readable description of the DDR Robot'''
        return "Mecanum Robot with wheel radius of {}, left to right length of {}, and front to back length of {}".format(self._rad, self._len_front, self._len_back)