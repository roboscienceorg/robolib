import numpy as np

class Sensor_Error(Exception):
    def __init__(self, message):
        self.message = message

class Sensor():
    '''
    This class represents
    '''

    def __init__(self, mean, cov ):
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

        self._mean = np.array(mean)
        self._cov = np.array(cov)

        n = self._mean.shape[0]

        if self._cov.shape[0] != n and self._cov.shape[1] != n:
            raise Sensor_Error("Mean and Covariance sizes are mismatched")

        self._dimension = n



    @property
    def mean(self):
        """
        The
        """
        return self._mean

    @mean.setter
    def mean(self, value):
        if value.shape[0] != self.dimension:
            raise Sensor_Error("Provided mean does not match the Sensor Dimension")

        else:
            self._mean = value

    @property
    def cov(self):
        """
        The
        """
        return self._cov

    @cov.setter
    def cov(self, value):
        if value.shape[0] != self.dimension or value.shape[1] != self.dimension:
            raise Sensor_Error("Provided covariance matrix does not match the Sensor Dimension")

        else:
            self._cov = value

    @property
    def dimension(self):
        """
        The
        """
        return self._dimension

    def sense( self, orig_data ):
        '''
        This function

        Parameters
        ----------

        Returns
        -------
        '''

        data = np.array(orig_data)

        if data.shape[0] != self.dimension:
            raise Sensor_Error("Data provided does not match sensor dimensions")

        else:
            dimension = self.dimension
            n = data.shape[1]
            samples = np.random.multivariate_normal(self.mean, self.cov, n)

            for i in range(dimension):
                for j in range(n):
                    data[i][j] = data[i][j] + samples[j][i]

            return data
