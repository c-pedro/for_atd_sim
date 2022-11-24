''' Rigid body dynamics model. '''

from typing import Tuple
import numpy as np
from scipy.linalg import inv
from for_atd_sim.lib_data.basic_data import IndexedData
from for_atd_sim.lib_math.so3 import skew_sym


def state_to_indexed(state : list) -> Tuple[IndexedData,IndexedData]:
    ''' Return list of attitudes and angular velocities in state. '''
    atd_data : IndexedData = {}
    anv_data : IndexedData = {}
    # parse rotations
    for body,i in enumerate([0,9,18]):
        atd_data[(0, body+1)] = np.array([state[i:i+9]]).reshape(3,3)
    # parse angular velocities
    for body,i in enumerate([27,30,33]):
        anv_data[(body+1, 0)] =np.array([state[i:i+3]]).reshape(3,1)

    return atd_data, anv_data


def continuous_formation_dynamics(t : float,
        x : np.ndarray,
        moi : IndexedData,
        tor_fun : callable,
        tor_par : dict
) -> np.ndarray:
    ''' Model for formation kinematics and dynamics. '''
    der_atd : IndexedData = {}
    der_anv : IndexedData = {}
    tor_val : IndexedData = {}
    dx = []

    # parse rotations and angular velocities
    atd, anv = state_to_indexed(x)

    # update torque
    for idx in [(0,1), (0,2), (0,3)]:
        tor_val[idx] = tor_fun(t, **{k:v.data[idx] for (k,v) in tor_par.items()})

    # compute derivatives
    for idx in [(0,1), (0,2), (0,3)]:
        anv_idx = (idx[1], idx[0])

        der_atd[idx] = atd[idx] @ skew_sym(anv[anv_idx])
        der_anv[anv_idx] = inv(moi[idx]) @ (tor_val[idx] - skew_sym(anv[anv_idx]) @ moi[idx] @ anv[anv_idx])

    # format output
    for idx in [(0,1), (0,2), (0,3)]:
        dx.extend(der_atd[idx].flatten())
    for idx in [(1,0), (2,0), (3,0)]:
        dx.extend(der_anv[idx].flatten())

    return dx


def continuous_angular_dynamics(t : float,
        x : np.ndarray,
        moi : np.ndarray,
        tor_fun : callable,
        tor_par : dict,
) -> np.ndarray:
    ''' Model for formation kinematics and dynamics. '''

    # initialize derivative array
    dx = []

    # parse rotations and angular velocities
    anv = np.array(x[:3]).reshape(3,1)

    # update torque
    tor_val = tor_fun(t, **tor_par)

    # compute derivatives
    der_anv = inv(moi) @ (tor_val - skew_sym(anv) @ moi @ anv)

    # format output
    dx = der_anv.flatten()

    return dx


def continuous_rbody_dynamics(
        t : float,
        x : np.ndarray,
        moi : np.ndarray,
        tor_fun : callable,
        tor_par : dict,
):
    ''' Differential equations for a rigid body.'''
    # initialize derivative array
    dx = []

    # parse rotations and angular velocities
    atd = x[:9].reshape(3,3)
    anv = x[9:].reshape(3,1)

    # update torque
    tor_val = tor_fun(t, **tor_par)

    # compute derivatives
    der_atd = atd @ skew_sym(anv)
    der_anv = inv(moi) @ (tor_val - skew_sym(anv) @ moi @ anv)

    # format output
    dx = np.concatenate([der_atd.reshape(9), der_anv.reshape(3)])

    return dx
