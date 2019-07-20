import numpy as np

class Sensor_Error(Exception):
    def __init__(self, message):
        self.message = message

class Sensor():
    '''
    This class represents a basic sensor. It allows for a user to provide
    an error profile and then take readings based on the actual measurements.
    This could be use in conjunction with a robot model to generate sensory
    data read from a simulated robot.
    '''

    def __init__(self, mean, cov ):
        '''
        This function instantiates the sensor. Currently, the sensor takes
        in simply a mean and covariance. These may be either numbers or 
        arrays for higher dimensional sensors.

        Parameters
        ----------
        mean:  Numeric or 1D Array of Numerics
            This is the mean of the Gaussian distribution which the sensor 
            errors will be pulled from. An array indicates a multi-variate
            distribution.
        cov:
            The covariance matrix of the Gaussian distribution which the 
            sensor erros will be pulled from. It must be symmetric and 
            positive-semidefinite for proper sampling (from Scipy).

        Returns
        -------
        A Sensor object
        '''

        try:
            self._mean = np.array(mean)
            self._cov = np.array(cov)
        except:
            return Sensor_Error("Either the mean or covariance could not be \
                    coerced to a NumPy array. Please check your values.")

        n = self._mean.shape[0]

        if self._cov.shape[0] != n and self._cov.shape[1] != n:
            raise Sensor_Error("Mean and Covariance sizes are mismatched")

        self._dimension = n


    @property
    def mean(self):
        """
        Getter for the error distribution mean.

        Returns
        -------
        mean:
            The mean of the distribution
        """
        return self._mean

    @mean.setter
    def mean(self, value):
        """
        Setter for the error distribution mean.

        Parameters
        ----------
        value:
            The value the mean(s) should be set to.

        Raises
        ------
        Sensor_Error:
            when the provided mean(s) don't match the sensor dimension.
        """
        if value.shape[0] != self.dimension:
            raise Sensor_Error("Provided mean(s) does not match \
                    the Sensor's dimension")

        else:
            self._mean = value

    @property
    def cov(self):
        """
        Getter for the error distribution covariance.

        Returns
        -------
        cov:
            The covariance of the distribution.
        """
        return self._cov

    @cov.setter
    def cov(self, value):
         """
        Setter for the error distribution covariance matrix.

        Parameters
        ----------
        value: Numeric, Array of Numeric
            The value the covariance(s) should be set to.

        Raises
        ------
        Sensor_Error:
            when the provided covariance matrix don't match the sensor dimension.
        """

        if value.shape[0] != self.dimension or value.shape[1] != self.dimension:
            raise Sensor_Error("Provided covariance matrix does not match the Sensor Dimension")

        else:
            self._cov = value

    @property
    def dimension(self):
        """
        Getter for the Sensor dimension.

        Returns
        -------
        dimension:
            The dimension of the Sensor.
        """
        return self._dimension

    def sense( self, orig_data ):
        '''
        This function allows the user to use the instantiated Sensor to
        sense actual data. This simply adds error values to the presented
        data based on the Sensor's error profile.

        Parameters
        ----------
        orig_data: Array of Numerics
            This is the data which the sensor should add error to. The dimensions
            of the data should be compatible with the Sensor's dimensions. Each
            row of the array is treated as another data point.

        Returns
        -------
        data:
            This is the new data with the errors added to each reading. This will
            be the same size as orig_data.

        Raises
        ------
        Sensor_Error:
            when the data is not compatible with the Sensor's dimensions.
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
