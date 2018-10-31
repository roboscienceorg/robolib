import numpy as np
from scipy import signal
import matplotlib.pyplot as plt


class ValToPWM:
    """
    This class helps a user plot a Pulse Width Modulation graph by taking in 
    the frequency and duty cycle.
    """
    def __init__(self, out, duty, freq=1.5, save=False):
        """
        Class contructor which initiazes the graph it's properties.
        
        Parameters
        ----------
        out:  String
            Name of output file.
        duty: Numeric
            Duty cycle
        freq: Numeric
            Frequency on the graph
        save: Boolean
            True is user wants to save the graph, False otherwise.

        Returns
        -------
        Graph object with the given parameters
        """
        self.out = out
        self.duty = duty/100
        self.freq = freq
        self.save = save
        
    def draw(self):
        """
        Draws the graph with the given duty cycle and frequency.
        """
        t = np.linspace(0, 1, 500, endpoint=False)
        plt.plot(t, signal.square(2 * np.pi * self.freq * t, duty=self.duty))
        plt.xlabel(str(self.duty*100) + "% duty cycle")
        plt.xticks([])
        plt.yticks([])
        
        plt.show()
        
        if self.save:
            plt.savefig("./" + str(self.out) + ".png",dpi=250,bbox_inches="tight",pad_inches=0.5)  
