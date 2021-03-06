import numpy as np

class DDR_Error(Exception):
    def __init__(self, message):
        self.message = message

class DDR():
    '''
    This class represents a differential drive robot. The user may construct
    the object with given wheel radii, axel length, and starting position.
    '''

    def __init__(self, rad_r, rad_l, L, starting_pos=(0.0, 0.0, 0.0)):
        '''
        This function creates and initializes the DDR with the given parameters.

        Parameters
        ----------
        rad_r:  Numeric
            The radius of the right wheel
        rad_l:  Numeric
            The radius of the left wheel
        L:      Numeric
            The length of the half-axel (Center of the axel to a wheel)
        starting_pos: list of 3 Numeric values
            This represents the [x, y, theta] of the robot's initial orientation. This
            value will default to [0, 0, 0]

        Returns
        -------
        DDR object with the given parameters
        '''
        self._rad_r = rad_r
        self._rad_l = rad_l
        self._L = L
        self._x = starting_pos[0]
        self._y = starting_pos[1]
        self._theta = starting_pos[2]

    @property
    def L(self):
        """
        The length from one wheel to the center of the axel. This can be set as follows: DDR.L = <new_value>
        """
        return self._L

    @L.setter
    def L(self, value):
        self._L = value

    @property
    def rad_r(self):
        """
        The radius of the right wheel. This can be set as follows: DDR.rad_r = <new_value>
        """

        return self._rad_r

    @rad_r.setter
    def rad_r(self, value):
        self._rad_r = value

    @property
    def rad_l(self):
        """
        The radius of the left wheel. This can be set as follows: DDR.rad_l = <new_value>
        """

        return self._rad_l

    @rad_l.setter
    def rad_l(self, value):
        self._rad_l = value

    @property
    def position(self):
        """
        The current position of the robot. This can be set as follows: DDR.position = <new_value> or
        individually with x, y, and theta properties.
        """

        return [self._x, self._y, self._theta]

    @position.setter
    def position(self, value):
        self._x = value[0]
        self._y = value[1]
        self._theta = value[2]

    @property
    def x(self):
        """
        The x position of the robot. This can be set as follows: DDR.x = <new_value>
        """

        return self._x

    @x.setter
    def x(self, value):
        self._x = value

    @property
    def y(self):
        """
        The y position of the robot. This can be set as follows: DDR.y = <new_value>
        """

        return self._y

    @y.setter
    def y(self, value):
        self._y = value

    @property
    def theta(self):
        """
        The angular orientation of the robot. This can be set as follows: DDR.theta = <new_value>
        """

        return self._theta

    @theta.setter
    def theta(self, value):
        self._theta = value


    def calc_global_velocities( self, phi_1, phi_2):
        '''
        This function takes in two angular velocities and uses them to compute the global
        x, y, and angular velocities of the robot.

        Parameters
        ----------
        phi_1:      Numeric
            This is the angular velocity of the right wheel
        phi_2:      Numeric
            This is the angular velocity of the left wheel

        Returns
        -------
        x_vel:      Numeric
            This is the velocity in the x-direction in the global scope

        y_vel:      Numeric
            This is the velocity in the y-direction in the global scope

        theta_vel: Numeric
            This is the velocity in the angular-direction in the global scope
        '''
        x_vel = (self._rad_r / 2.0) * (phi_1 + phi_2) * np.cos(self._theta)
        y_vel = (self._rad_r / 2.0) * (phi_1 + phi_2) * np.sin(self._theta)
        theta_vel = (self._rad_r / (2.0 * self._L)) * (phi_1 - phi_2)

        return x_vel, y_vel, theta_vel


    def FK( self, ang_vel1, ang_vel2, t, as_positions=False ):
        """
        This function takes in a list of angular velocities along with the time that
        each velocity occurs. It then computes a list of the positions of the robot and
        returns this list.

        NOTE: This equation will only return correct results if the wheels are the same
        size.

        Parameters
        ----------
        ang_vel1:       list(Numeric) length n
            This is the list of angular velocities of the right wheel.
        ang_vel2:       list(Numeric) length n
            This is the list of angular velocities of the left wheel.
        t:              list(Numeric) length n
            This is the list of times of the provided angular velocities.

        Returns
        -------
        pos_list:       list( list(Numeric) length 3) length n
            This is the list of positions computed from the provided parameters. This
            list is a list of lists of length 3 structures as [x, y, theta].
        """
        if not (len(ang_vel1) == len(ang_vel2) == len(t)):
            raise DDR_Error("Arrays must be of the same length")

        pos_list = []

        for i in range(len(ang_vel1)):
            x_vel, y_vel, theta_vel = self.calc_global_velocities(ang_vel1[i], ang_vel2[i])

            self._x = self._x + x_vel*t[i]
            self._y = self._y + y_vel*t[i]
            self._theta = self._theta + theta_vel*t[i]

            pos_list.append([self._x, self._y, self._theta])
        if as_positions == True:
            return pos_list

        else:
            x = [i[0] for i in pos_list]
            y = [i[1] for i in pos_list]
            theta = [i[2] for i in pos_list]

            return x, y, theta

    def IK(self, x_vel, y_vel, x_accel, y_accel):
        """
        This function takes in a list of x and y velocities as well as their accelerations
        to compute the angular velocities of the wheels required to traverse this path.

        It returns these velocities as two lists (combined into a tuple).

        NOTE: This equation will only return correct results if the wheels are the same
        size.

        Parameters
        ----------
        x_vel:      list(Numeric) length n
            This is the list of x velocities at each timestep
        y_vel:      list(Numeric) length n
            This is the list of y velocities at each timestep
        x_accel:    list(Numeric) length n
            This is the list of x acceleration at each timestep
        y_accel:    list(Numeric) length n
            This is the list of y acceleration at each timestep

        Returns
        -------
        tuple:       tulple( list(Numeric) length n, list(Numeric) length n)
            This tuple contains two elements. The first is a list of the left wheel velocity
            at each timestep and the second is the list of right wheel velocity.

        """

        v = np.sqrt( np.power(x_vel, 2) + np.power(y_vel, 2) )
        k = x_vel * y_accel - y_vel * x_accel / np.power(v, 3)
        phi_1 = (self._L / self._rad_r) * ((x_vel * y_accel - y_vel * x_accel) / (np.power(x_vel, 2) + np.power(y_vel, 2))) + v / self._rad_r
        phi_2 = -(self._L / self._rad_r) * ((x_vel * y_accel - y_vel * x_accel) / (np.power(x_vel, 2) + np.power(y_vel, 2))) + v / self._rad_r

        return (phi_1, phi_2)

    def __str__(self):
        """Provides a human readable description of the DDR Robot"""
        return "DDR Robot with wheel radii {} and {}, and L {}".format(self._rad_r, self._rad_l, self._L)


