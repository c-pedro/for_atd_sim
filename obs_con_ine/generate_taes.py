import numpy as np
from for_atd_sim.lib_math.random import random_unit_3d_vec
from for_atd_sim.lib_math.so3 import rot_mtx
from for_atd_sim.obs_con_ine.keys import ObsModelKeys
from for_atd_sim.obs_con_ine.keys import kpro
import for_atd_sim.lib_atd.torque as torque


def taes_dynamics() -> dict:
    ''' Set input values of moment of inertia and torque. '''
    input_dyn = {}
    moi_vehicle = np.diag([70, 70, 60])

    input_dyn[kpro.ProKeys.moi] = {
        (0,1) : moi_vehicle,
        (0,2) : moi_vehicle,
        (0,3) : moi_vehicle,
    }

    input_dyn[torque.KEY_TOR_FUN] = {None : torque.KEY_TORFUN_SIN}
    input_dyn[torque.KEY_TOR_PAR_INI] = {
        (0,1) : np.array([[1],[0],[0]]),
        (0,2) : np.array([[0],[1],[0]]),
        (0,3) : np.array([[0],[0],[1]]),
    }
    input_dyn[torque.KEY_TOR_PAR_GAIN] = {
        (0,1) : np.pi/180,
        (0,2) : np.pi/180,
        (0,3) : np.pi/180,       
    }
    input_dyn[torque.KEY_TOR_PAR_FREQ] = {
        (0,1) : 1,
        (0,2) : 1,
        (0,3) : 1,
    }
    input_dyn[torque.KEY_TOR_PAR_PHASE] = {
        (0,1) : np.array([[0]]),
        (0,2) : np.array([[0]]),
        (0,3) : np.array([[0]]),
    }
    
    return input_dyn


def taes_sensors() -> dict:
    input_sensors = {}
    input_sensors[kpro.SenModelKeys.anv_sdev] = {
        (1,0) : 5*10**-6,
        (2,0) : 5*10**-6,
        (3,0) : 5*10**-6,
    }
    input_sensors[kpro.SenModelKeys.los_sdev] = {
        (1,2) : 80*10**-6,
        (2,1) : 80*10**-6,
        (1,3) : 80*10**-6,
        (3,1) : 80*10**-6,
    }
    input_sensors[kpro.SenModelKeys.ref_sdev] = {
        (1,1) : 80*10**-6,
        (2,2) : 80*10**-6,
        (3,3) : 80*10**-6,
    }

    return input_sensors


def taes_obs() -> dict:
    input_obspar = {}
    input_obspar[ObsModelKeys.weight_kin] = {
        (1,0) : [(0, [[1.2]])],
        (2,0) : [(0, [[1.2]])],
        (3,0) : [(0, [[1.2]])],
        
    }
    input_obspar[ObsModelKeys.weight_pot] = {
        (1,1) : [(0, [[0.6]])],
        (2,2) : [(0, [[0.6]])],
        (3,3) : [(0, [[0.6]])],
        (1,2) : [(0, [[0.6]])],
        (2,1) : [(0, [[0.6]])],
        (1,3) : [(0, [[0.6]])],
        (3,1) : [(0, [[0.6]])],
        # for atd obs
        (0,1) : [(0, [[0.6]])],
        (0,2) : [(0, [[0.6]])],
        (0,3) : [(0, [[0.6]])],
    }
    input_obspar[ObsModelKeys.obs_damp] = {
        (1,0) : [(0, [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])],
        (2,0) : [(0, [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])],
        (3,0) : [(0, [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])],
    }
    return input_obspar


def taes_maneuver() -> dict:
    input_manv = {}

    time_i = 0
    time_f = 1000 # lasts for all trials
    
    axis_2 = random_unit_3d_vec()
    axis_3 = random_unit_3d_vec()
    dangle_2 = np.random.default_rng().uniform(0,2*np.pi/300)
    dangle_3 = np.random.default_rng().uniform(0,2*np.pi/300)

    input_manv[kpro.ProModelKeys.manv] = {
        (1,2) : [[time_i, time_f, dangle_2, axis_2]],
        (1,3) : [[time_i, time_f, dangle_3, axis_3]]
    }
    
    return input_manv