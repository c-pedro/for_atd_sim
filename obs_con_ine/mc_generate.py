''' Custom analysis functions '''
import numpy as np
from for_atd_sim.pro_3hetfor.generate import random_bounded_anv
from for_atd_sim.pro_3hetfor.generate import random_configuration
from for_atd_sim.obs_con_ine.generate import get_initial_obs_zeroanv
from for_atd_sim.obs_con_ine.generate_taes import taes_dynamics
from for_atd_sim.obs_con_ine.generate_taes import taes_maneuver
from for_atd_sim.obs_con_ine.generate_taes import taes_obs
from for_atd_sim.obs_con_ine.generate_taes import taes_sensors


def custom_generate() -> dict:
    new_input = {}

    # random configuration
    new_input.update(**random_configuration())
    new_input.update(**random_bounded_anv())
    # new_input.update(**random_maneuver()) #TODO
    # TODO : random obs

    return new_input


def taes_random_fixed_error(max_err_deg = 30, max_anv_deg=10):
    ''' Return initial parameters of random configuration with fixed maximum error.'''    
    new_input = {}

    # random configuration
    new_input.update(**random_configuration())
    new_input.update(**random_bounded_anv(np.deg2rad(max_anv_deg)))
    
    # taes parameters
    new_input.update(**taes_obs())
    new_input.update(**taes_dynamics())
    new_input.update(**taes_maneuver())
    new_input.update(**taes_sensors())

    # sample error fro
    err_deg_db = {
        (0,1) : np.random.default_rng().uniform(0, max_err_deg),
        (0,2) : np.random.default_rng().uniform(0, max_err_deg),
        (0,3) : np.random.default_rng().uniform(0, max_err_deg),
    }
    err_deg_db[(0,np.random.default_rng().choice([1,2,3]))] = max_err_deg
    
    new_input = get_initial_obs_zeroanv(new_input, err_deg_db)

    return new_input


def random_fixed_error(max_err_deg = 30):
    ''' Return initial parameters of random configuration with fixed maximum error.'''    
    new_input = {}

    # random configuration
    new_input.update(**random_configuration())
    new_input.update(**random_bounded_anv())
    # new_input.update(**random_maneuver()) #TODO

    # sample error fro
    err_deg_db = {
        (0,1) : np.random.default_rng().uniform(0, max_err_deg),
        (0,2) : np.random.default_rng().uniform(0, max_err_deg),
        (0,3) : np.random.default_rng().uniform(0, max_err_deg),
    }
    err_deg_db[(0,np.random.default_rng().choice([1,2,3]))] = max_err_deg
    
    new_input = get_initial_obs_zeroanv(new_input, err_deg_db)

    return new_input

