import numpy as np

class SimSensors_Error(Exception):
    def __init__(self, message):
        self.message = message

class SimSensors():
    '''
    This class represents 
    '''

    def __init__(self, sensor_set, n, m ):
        '''
        This function 

        Parameters
        ----------
        sensor_set:  Tupple
            2D list containing the set of sensor information 
        n:
            The number of sensors
        m: 
            The number of readings from the sensors
 
        Returns
        -------
        SimSensors object with the given parameters
        '''
        self._sensor_set = sensor_set
        self._n = n
        self._m = m


    @property
    def sensor_set(self):
        """
        The 
        """
        return self._sensor_set

    @sensor_set.setter
    def sensor_set(self, value):
        self._sensor_set = value

    @property
    def n(self):
        """
        The 
        """
        return self._n

    @n.setter
    def n(self, value):
        self._n = value

    @property
    def m(self):
        """
        The 
        """
        return self._m

    @m.setter
    def m(self, value):
        self._m = value


    def UniformFusion( self, sensor_set, x):
        '''
        This function 

        Parameters
        ----------
        sensor_set:      2D list
            This 
        x:      Numeric
            This 

        Returns
        -------
        fuse:      list
            This 
        '''


    def SensorFusion( self, sensor_set, x):
        '''
        This function 

        Parameters
        ----------
        sensor_set:      2D list
            This 
        x:      Numeric
            This 

        Returns
        -------
        fuse:      list
            This 
        '''


