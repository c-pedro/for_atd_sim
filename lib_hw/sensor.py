
import numpy as np


class SensorModel():
    ''' Base Class for Sensor Models'''

    def __init__(self) -> None:
        self.cov = np.zeros((3,3))


    def measure(self, true_val : np.ndarray) -> np.ndarray:
        ''' Returns measurement from true value.
        
            To override.
        '''
        return true_val 


   