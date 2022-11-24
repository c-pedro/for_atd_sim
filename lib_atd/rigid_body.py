''' Module with rigid body dynamics functions.

'''
from copy import deepcopy
import numpy as np

from scipy.linalg import expm
from scipy.integrate import solve_ivp

from for_atd_sim.lib_data import DataSeries
from for_atd_sim.lib_data import AttitudeData
from for_atd_sim.lib_data import AngularVelocityData

import for_atd_sim.core.keys as km
import for_atd_sim.lib_atd.torque as tor

from for_atd_sim.lib_math.so3 import skew_sym
import for_atd_sim.lib_atd.model_rbody as dynmodel


# TODO in own module
def get_next_atd_dis(
        dt : float,
        atd : np.ndarray,
        anv : np.ndarray
    ) -> np.ndarray:
    ''' Return the next attitude from the discrete model.

        Arguments
        ---------
        dt : float
            Time step.
        atd : np.ndarray
            Current attitude.
        anv : np.ndarray
            Current angular velocity.

        Returns
        -------
        ndarray
            Value of the attitude at time t+dt.

    '''
    return atd @ expm(dt * skew_sym(anv))


def statetime_to_dict(state_time, time_vec) -> dict[str,DataSeries]:
    atd_series = DataSeries()
    anv_series = DataSeries()

    # transform data
    for i in range(len(state_time.T)):
        atd_pnt = AttitudeData(time_vec[i])
        anv_pnt = AngularVelocityData(time_vec[i])
        
        atd_indexed, anv_indexed = dynmodel.state_to_indexed(state_time.T[i])

        for idx in [(0,1), (0,2), (0,3)]:
            anv_idx = (idx[1], idx[0])
            atd_pnt.data[idx] = atd_indexed[idx]
            anv_pnt.data[anv_idx] = anv_indexed[anv_idx]
    
        atd_pnt.complete_data()

        # append to series
        atd_series.series.append(deepcopy(atd_pnt))
        anv_series.series.append(deepcopy(anv_pnt))

    out_dict = {
        km.ProKeys.atd : atd_series,
        km.ProKeys.anv : anv_series
    }

    return out_dict


def get_formation_dynamics(tvec : list, ini_data : dict, true_par : dict) -> dict:
    ''' Return true values attitude and angular velocity. '''

    # parse max step
    my_max_step = true_par.get(km.ProKeys.ivp_max_step, 0.001)

    # parse moment of inertia
    moi_val = true_par[km.ProKeys.moi].data
    
    # parse torque
    tor_fun = true_par[tor.KEY_TOR_FUN]
    tor_par = {key : true_par[key] for key in tor.TOR_PAR_SET}

    # initialize ivp variable
    ivp_var = []
    # add attitutde to ivp variable
    for idx in [(0,1), (0,2), (0,3)]:
        ivp_var.extend(ini_data[km.ProKeys.atd].data[idx].flatten())
    # anv attitutde to ivp variable
    for idx in [(1,0), (2,0), (3,0)]:
        ivp_var.extend(ini_data[km.ProKeys.anv].data[idx].flatten())

    # compute attitude and angular velocity
    ivp_res = solve_ivp(
        fun = dynmodel.continuous_formation_dynamics,
        t_span = [tvec[0], tvec[-1]],
        y0 = ivp_var,
        t_eval = tvec,
        args = (moi_val, tor_fun, tor_par),
        max_step = my_max_step
        # atol=10**-11,
        # rtol=10**-11,
    )

    if ivp_res.success:
        return statetime_to_dict(ivp_res.y, tvec) 

    raise ValueError
