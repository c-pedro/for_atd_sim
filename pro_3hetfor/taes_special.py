''' Functions that generate a specific type of problem. '''

import numpy as np
import for_atd_sim.lib_math.random as rnd
import for_atd_sim.pro_3hetfor.keys as kpkg
from for_atd_sim.lib_math import so3
from typing import Tuple
from for_atd_sim.pro_3hetfor.generate import random_configuration


def taes_degenerate_configuration(deg_index_1, deg_index_2):
    ''' Returns a specific degenerate random configuration. '''
    my_config = random_configuration()

    my_config[kpkg.ProModelKeys.ilos][(1,2)] = np.array([[1],[1],[0]])
    my_config[kpkg.ProModelKeys.ilos][(1,3)] = np.array([[0],[1],[1]])
    
    my_config[kpkg.ProModelKeys.ref][(1,0)] = np.array([[0],[0],[1]])
    my_config[kpkg.ProModelKeys.ref][(2,0)] = np.array([[0],[1],[0]])
    my_config[kpkg.ProModelKeys.ref][(3,0)] = np.array([[1],[0],[0]])
    
    if deg_index_1 == (1,0):
        if deg_index_2 == (3,0):
            my_config[kpkg.ProModelKeys.ref][deg_index_1] = my_config[kpkg.ProModelKeys.ref][deg_index_2]
        elif deg_index_2 == (1,3):
            my_config[kpkg.ProModelKeys.ref][deg_index_1] = my_config[kpkg.ProModelKeys.ilos][deg_index_2]
        else:
            raise IndexError
    else: 
        raise IndexError


    return my_config


def taes_ambiguous_configuration():
    ''' Returns a specific ambiguous random configuration. '''
    my_config = random_configuration()

    my_config[kpkg.ProModelKeys.ilos][(1,2)] = np.array([[1],[1],[0]])
    my_config[kpkg.ProModelKeys.ilos][(1,3)] = np.array([[0],[1],[1]])
    
    my_config[kpkg.ProModelKeys.ref][(1,0)] = np.array([[0],[0],[1]])
    my_config[kpkg.ProModelKeys.ref][(2,0)] = np.array([[0],[1],[0]])
    my_config[kpkg.ProModelKeys.ref][(3,0)] = np.array([[1],[0],[0]])

    alpha = 0
    beta = 0
    gamma = 0

    n3 = get_normal_vector(my_config[kpkg.ProModelKeys.ref][(1,0)], my_config[kpkg.ProModelKeys.ref][(3,0)])
    n13 = get_normal_vector(my_config[kpkg.ProModelKeys.ref][(1,0)], my_config[kpkg.ProModelKeys.ilos][(1,3)])

    my_config[kpkg.ProModelKeys.ref][(2,0)] = so3.rot_mtx(alpha, my_config[kpkg.ProModelKeys.ref][(1,0)]) @ so3.rot_mtx(beta, n3) @ my_config[kpkg.ProModelKeys.ref][(3,0)]
    my_config[kpkg.ProModelKeys.ilos][(1,2)] = so3.rot_mtx(alpha, my_config[kpkg.ProModelKeys.ref][(1,0)]) @ so3.rot_mtx(gamma, n13) @ my_config[kpkg.ProModelKeys.ilos][(1,3)]

    return my_config


def get_normal_vector(vec1, vec2):
    return so3.skew_sym(vec1) @ vec2 / np.linalg.norm(so3.skew_sym(vec1) @ vec2)



