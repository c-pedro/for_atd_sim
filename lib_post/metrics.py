import numpy as np
from for_atd_sim.lib_math.so3 import skew_sym


def get_vec_align_score(vec1 : np.ndarray, vec2 : np.ndarray) -> float:
    ''' Return alignment score between to vectors. '''
    return 1 - abs(vec1.T @ vec2)


def get_alpha(a : np.ndarray, b : np.ndarray, c : np.ndarray) -> float:

    y = a.T @ skew_sym(b) @ c
    x = a.T @ skew_sym(b) @ skew_sym(b) @ c

    return - np.arctan2(y,x)


def get_coplanar_proximity(a : np.ndarray, b : np.ndarray, c : np.ndarray) -> float:
    ''' Returns a scalar metric of coplanarity of 3 vectors. '''
    return abs(a.T @ skew_sym(b) @ c)
