''' Tools for error computation and analysis in attitude.
'''
# third-party libraries
import numpy as np


def compute_error_rotation(true_rot : np.ndarray, estimated_rot : np.ndarray) -> np.ndarray:
    ''' Returns error rotation matrix. '''
    return true_rot @ estimated_rot.T


def compute_principal_angle(rot : np.ndarray) -> np.ndarray:
    ''' Returns principal angle of a given rotation. 

    
    '''
    # orthogonalize rotation
    if not abs((np.trace(rot) - 1) / 2 ) <= 1:
        u_mat, _, vh_mat = np.linalg.svd(rot)
        rot = u_mat @ vh_mat
        
    if abs((np.trace(rot) - 1) / 2 ) <= 1:
        return np.arccos((np.trace(rot) - 1) / 2)

    raise ValueError
   