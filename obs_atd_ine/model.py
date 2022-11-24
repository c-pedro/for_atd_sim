''' Complete formation model.

'''
import numpy as np
from for_atd_sim.lib_math.so3 import skew_sym
import for_atd_sim.obs_con_ine.keys as kpkg
from for_atd_sim.core.sensors import Sensors
from for_atd_sim.core.simmodel import SimModel
from for_atd_sim.core.simsettings import SimSettings
from for_atd_sim.pro_3hetfor.pro_model import setup_problem
from for_atd_sim.pro_3hetfor.sen_model import setup_sensors
from for_atd_sim.obs_atd_ine.obs_model import setup_observer


def setup_model(smodel : SimModel, input_data : dict, settings : SimSettings):
    
    setup_problem(smodel, input_data, settings)

    setup_sensors(smodel, input_data, settings)
    
    setup_observer(smodel, input_data, settings)



def state_to_list(state):
    ''' Return list of attitudes and angular velocities in state. '''

    # initialize rotations and angular velocities lists
    atd_pro = [None]
    atd_obs = [None]
    anv_pro = [None]
    anverr_obs = [None]

    # TODO : function in lib_atd
    # parse rotations
    for i in [0,9,18]:
        atd_pro.append(np.array([state[i:i+9]]).reshape(3,3))
    for i in [27,36,45]:
        atd_obs.append(np.array([state[i:i+9]]).reshape(3,3))

    # TODO : function in lib_atd
    # parse angular velocities
    for i in [54,57,60]:
        anv_pro.append(np.array([state[i:i+3]]).reshape(3,1))
    for i in [63,66,69]:
        anverr_obs.append(np.array([state[i:i+3]]).reshape(3,1))

    return atd_pro, atd_obs, anv_pro, anverr_obs


def list_to_state(atd_pro, atd_obs, anv_pro, anverr_obs):
    ''' Return list of attitudes and angular velocities in state. '''

    # initialize state
    state = []

    for i in range(1,4):
        state.extend(atd_pro[i].flatten())
    for i in range(1,4):
        state.extend(atd_obs[i].flatten())
    for i in range(1,4):
        state.extend(anv_pro[i].flatten())
    for i in range(1,4):
        state.extend(anverr_obs[i].flatten())

    return state


# def complete_model(
#         time : float,
#         state : np.ndarray,
#         moi : np.ndarray,
#         tor_fun : callable,
#         tor_par : dict,
#         pot_fun : callable,
#         pot_par : dict,
#         obs_par : dict,
#         true_init : dict,
#         sensors : Sensors
#     ) -> np.ndarray:
#     ''' Constrained observer model. '''
#     # initialize derivatives
#     der_atd_pro = [None]
#     der_anv_pro = [None]
#     der_atd_obs = [None]
#     der_anverr_obs = [None]

#     # parse variables from state
#     atd_pro, atd_obs, anv_pro, anverr_obs = state_to_list(state)

#     # parse obs parameters
#     weight_kin = obs_par[kpkg.ObsKeys.weight_kin].data
#     damping = obs_par[kpkg.ObsKeys.obs_damp].data

#     # compute torque
#     tor_val = np.empty(4, object)
#     for i in [1,2,3]:
#         tor_val[i] = tor_fun(time, **tor_par[i])

#     # compute measurements with noise # TODO : if noise flag active
#     noise = False
#     if noise:
#         anv_pro_aux = np.atleast_2d(np.array(anv_pro, object)).T # FIXME
#         anv_sen_aux, _ = sensors.measure_key(kpkg.SenKeys.anv, anv_pro_aux)
#         # TODO format anv pro input
#         anv_sen = list(anv_sen_aux.T[0])
#     else:
#         anv_sen = anv_pro

#     # compute attitude derivatives
#     for i in range(1,4):
#         der_atd_pro.append(atd_pro[i] @ skew_sym(anv_pro[i]))
#         der_atd_obs.append(atd_obs[i] @ skew_sym(anv_sen[i] - anverr_obs[i]))

#     # compute potential
#     pot_val = np.empty(4, object)
#     pot_val = pot_fun(time, atd_pro, atd_obs, true_init, sensors, **pot_par)

#     # compute angular velocity and error derivatives
#     for i in range(1,4):
#         der_anv_pro.append(
#             np.linalg.inv(moi[i]) @ (
#                 tor_val[i]
#                 - skew_sym(anv_pro[i]) @ moi[i] @ anv_pro[i]
#             )
#         )
#         der_anverr_obs.append(
#             pot_val[i]
#             - (weight_kin[i,i] * skew_sym(anv_sen[i]) + damping[i,i])
#             @ anverr_obs[i]
#         )

#     # format derivative
#     der_state = list_to_state(
#         der_atd_pro, der_atd_obs,
#         der_anv_pro, der_anverr_obs)

#     return der_state
