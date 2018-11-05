import numpy as np
from scipy import signal
import matplotlib.pyplot as plt


class MyError(Exception):
    def __init__(self, message):
        super().__init__(message)


class ValToPWM:
    '''
    This class helps a user plot a Pulse Width Modulation graph by taking in 
    the frequency and duty cycle.
    '''
    def __init__(self, duty, freq=1.5, save=False, output="test.png"):
        '''
        Class contructor which initiazes the graph's varaibles. 
        
        Parameters
        ----------
        duty: Numeric
            Duty cycle in percentage
        freq: Numeric
            Frequency on the graph
        save: Boolean
            True is user wants to save the graph, False otherwise
        output: String
            Name of output file

        Returns
        -------
        Graph object with the given parameters
        '''
        if duty < 0 or duty > 100:
            raise MyError("Duty cycle is a percentage and should be between 0 and 100")
            
        self.duty = duty/100
        self.freq = freq
        self.save = save
        self.output = output
        
    def draw(self):
        '''
        Draws the graph with the given duty cycle and frequency.
        '''
        t = np.linspace(0, 1, 500, endpoint=False)
        plt.plot(t, signal.square(2 * np.pi * self.freq * t, duty=self.duty))
        plt.xlabel(str(self.duty*100) + "% duty cycle")
        plt.xticks([])
        plt.yticks([])
        
        plt.show()
        
        if self.save:
            plt.savefig("./" + str(self.output),dpi=250,bbox_inches="tight",pad_inches=0.5)  
