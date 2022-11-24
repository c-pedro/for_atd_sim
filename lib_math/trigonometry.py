import numpy as np

def interval_0_2pi(angle):
    ''' Returns a given angle in the interval [0, 2pi].
    '''
    return angle - 2.0 * np.pi * np.floor(angle / (2.0 * np.pi) )


def interval_pm_pi(angle):
    ''' Returns a given angle in the interval [-pi, pi].
    '''
    return angle - 2.0 * np.pi * np.floor((angle + np.pi) / (2.0 * np.pi) )


