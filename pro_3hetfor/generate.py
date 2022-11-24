''' Functions that generate a specific type of problem. '''

import numpy as np
import for_atd_sim.lib_math.random as rnd
import for_atd_sim.pro_3hetfor.keys as kpkg
from for_atd_sim.lib_math import so3
from typing import Tuple

def random_configuration():
    ''' Return a random configuration. '''
    # generate random attitude
    atd_arr = {}
    atd_arr[(0,1)] = rnd.random_atd()
    atd_arr[(0,2)] = rnd.random_atd()
    atd_arr[(0,3)] = rnd.random_atd()

    # generate random LOS
    ilos_arr = {}
    ilos_arr[(1,2)] = rnd.random_unit_3d_vec()
    ilos_arr[(1,3)] = rnd.random_unit_3d_vec()

    # generate random reference
    ref_arr = {}
    ref_arr[(1,0)] = rnd.random_unit_3d_vec()
    ref_arr[(2,0)] = rnd.random_unit_3d_vec()
    ref_arr[(3,0)] = rnd.random_unit_3d_vec()

    config_db = {
        kpkg.ProModelKeys.atd : atd_arr,
        kpkg.ProModelKeys.ref : ref_arr,
        kpkg.ProModelKeys.ilos : ilos_arr,
    }

    return config_db


def random_bounded_anv(bound_rad = 1.0):
    ''' Return angular velocity with normal bounded by input. '''
    anv_arr = {}

    for index in [(1,0),(2,0),(3,0)]:
        
        # sample magnitude
        mag = np.random.default_rng().uniform(0, bound_rad)
        
        # sample angular velocity
        anv_arr[index] = mag * rnd.random_unit_3d_vec()

    return {kpkg.ProModelKeys.anv : anv_arr}


def gen_coplanar_vector(vec_a, vec_b):
    '''Return random vector in the same plane as input vectors. '''
    angle = np.random.default_rng().random() * 2 * np.pi
    axis = so3.skew_sym(vec_a) @ vec_b
    axis = axis / np.linalg.norm(axis)

    return so3.rot_mtx(angle, axis) @ vec_a


def random_degenerate_configuration(deg_index_1, deg_index_2):
    ''' Returns a specific degenerate random configuration. '''
    ref_idx_list = [(1,0),(2,0),(3,0)]
    ilos_idx_list = [(1,2),(1,3)]
    
    my_config = random_configuration()

    if deg_index_1 in ref_idx_list:
        if deg_index_2 in ref_idx_list:
            my_config[kpkg.ProModelKeys.ref][deg_index_1] = my_config[kpkg.ProModelKeys.ref][deg_index_2]
        
        elif deg_index_2 in ilos_idx_list:
            my_config[kpkg.ProModelKeys.ref][deg_index_1] = my_config[kpkg.ProModelKeys.ilos][deg_index_2]

        else:
            raise IndexError



    elif deg_index_1 in ilos_idx_list:
        if deg_index_2 in ref_idx_list:
            my_config[kpkg.ProModelKeys.ilos][deg_index_1] = my_config[kpkg.ProModelKeys.ref][deg_index_2]
        
        elif deg_index_2 in ilos_idx_list:
            raise IndexError

        else:
            raise IndexError


    else:
        raise IndexError

    return my_config


def random_degenerate_maneuver(
    index : Tuple[int,int], 
    itime_interval : list, 
    ftime_interval : list, 
    angle_interval : list,
    **other
) -> dict[Tuple[int,int],list]:
    ''' Returns a random maneuver for a given index. 
    '''
    itime = np.random.default_rng().uniform(itime_interval[0], itime_interval[1])
    ftime = np.random.default_rng().uniform(max([ftime_interval[0], itime]), ftime_interval[1])
    angle = np.random.default_rng().uniform(angle_interval[0], angle_interval[1])
    axis = other.get('axis', rnd.random_unit_3d_vec())

    return {index : [[itime, ftime, angle, axis]]}


def random_ambiguous_configuration():
    ''' Returns a specific ambiguous random configuration. '''
    atd_arr = {}
    ilos_arr = {}
    ref_arr = {}
    rnd_gen = np.random.default_rng()

    atd_arr[(0,1)] = rnd.random_atd()
    atd_arr[(0,2)] = rnd.random_atd()
    atd_arr[(0,3)] = rnd.random_atd()

    alpha = 2 * np.pi * rnd_gen.random()
    beta = 2 * np.pi * rnd_gen.random()
    gamma = 2 * np.pi * rnd_gen.random()

    ref_arr[(1,0)] = rnd.random_unit_3d_vec()
    ref_arr[(3,0)] = rnd.random_unit_3d_vec()
    ilos_arr[(1,3)] = rnd.random_unit_3d_vec()

    n3 = get_normal_vector(ref_arr[(1,0)], ref_arr[(3,0)])
    n13 = get_normal_vector(ref_arr[(1,0)], ilos_arr[(1,3)])

    ref_arr[(2,0)] = -so3.rot_mtx(alpha, ref_arr[(1,0)]) @ so3.rot_mtx(beta, n3) @ ref_arr[(3,0)]
    ilos_arr[(1,2)] = so3.rot_mtx(alpha, ref_arr[(1,0)]) @ so3.rot_mtx(gamma, n13) @ ilos_arr[(1,3)]

    my_config = {
        kpkg.ProModelKeys.atd : atd_arr,
        kpkg.ProModelKeys.ref : ref_arr,
        kpkg.ProModelKeys.ilos : ilos_arr,
    }

    return my_config


def get_normal_vector(vec1, vec2):
    return so3.skew_sym(vec1) @ vec2 / np.linalg.norm(so3.skew_sym(vec1) @ vec2)




# TODO : refresh
# def gen_coplanar_branches() -> dict:
#     ''' Retrun problem with coplanar branches. '''

#     n_body = 3
#     # setup attitude
#     atd_list = gen_rnd_atd(n_body)
#     atd_list = np.array([atd_list], object)

#     n_body = 3
#     rnd_gen = np.random.default_rng()

#     i1 = 2 * rnd_gen.random((n_body, 1)) - 1
#     i2 = 2 * rnd_gen.random((n_body, 1)) - 1
#     i3 = 2 * rnd_gen.random((n_body, 1)) - 1

#     i1 = i1 / np.linalg.norm(i1)
#     i2 = i2 / np.linalg.norm(i2)
#     i3 = i3 / np.linalg.norm(i3)

#     # change inertial values
#     i12 = gen_coplanar_vector(i1, i2)
#     i13 = gen_coplanar_vector(i1, i3)


#     ref_list = np.array([[None],[i1],[i2],[i3]], object)
#     ilos_list = np.array([
#         [None, None, None, None],
#         [None, None, +i12, +i13],
#         [None, -i12, None, None],
#         [None, -i13, None, None]], object)

#     # setup dict
#     pro_db = {
#         kpkg.ProModelKeys.atd : atd_list,
#         kpkg.ProModelKeys.ilos : ilos_list,
#         kpkg.ProModelKeys.ref : ref_list
#     }

#     return pro_db

