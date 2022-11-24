import numpy as np
from for_atd_sim.lib_math.random import random_unit_3d_vec
from for_atd_sim.lib_math.so3 import rot_mtx
from for_atd_sim.obs_con_ine.keys import ObsModelKeys
from for_atd_sim.obs_con_ine.keys import kpro


def get_initial_obs_zeroanv(input_config : dict, error_deg : dict) -> dict:
    ''' Returns updated configuration with observer initial value. 
    
        Fixed initial attitude observer error with random axis.
        Zero angular velocity error.
    '''
    input_config[ObsModelKeys.atd] = {}
    input_config[ObsModelKeys.anv] = {}

    for index in [(0,1), (0,2), (0,3)]:
        pro_atd_ini = input_config[kpro.ProModelKeys.atd][index]
        obs_atd_err = rot_mtx(np.deg2rad(error_deg[index]), random_unit_3d_vec())
        obs_atd_ini = obs_atd_err.T @ pro_atd_ini

        input_config[ObsModelKeys.atd][index] = obs_atd_ini

    for index in [(1,0), (2,0), (3,0)]:
        input_config[ObsModelKeys.anv][index] = input_config[kpro.ProModelKeys.anv][index]

    return input_config
