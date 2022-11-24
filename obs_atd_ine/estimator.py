'''
Discrete variational attitude observer functions.

Lagrangian is applied to each individual vehicle in a heterogeneous formation.

Functions
---------
anv_observer
    Residual of discretized observer internal angular velocity dynamics.
next_atd_estimate
    Return next attitude estimate from discrete kinematics.
next_obs_feedback
    Return next observer feedback from its dynamics residual function.
compute_measured_rotation
    Compute rotation matrix from measurement data.
variational_estimator_dsc_ind
    Return estimated attitude and observer internal state.

'''

# general
import numpy as np
from scipy import optimize as opt
from scipy.linalg import expm
# libs
from for_atd_sim.lib_math.so3 import skew_sym, inv_skew_sym, orthogonalize_mat
from for_atd_sim.lib_data.centralized_data import AttitudeData
from for_atd_sim.lib_data.centralized_data import AngularVelocityData
from for_atd_sim.lib_atd.rigid_body import get_next_atd_dis
# SimCell
from for_atd_sim.core.sensors import Sensors
import for_atd_sim.core.observer as obs
# subpackage
import for_atd_sim.obs_atd_ine.keys as kpkg
# deterministic solution
from for_atd_sim.obs_detatd_inerel.algorithm import compute_solution


from typing import Tuple
from for_atd_sim.lib_data.basic_data import DataPoint


def anv_observer(nPhi, oPhi, time_step, weight_kin, weight_pot, mDamp, nSenAnv, nPotErr):
    '''
    Residual of discretized observer internal angular velocity dynamics.

    Parameters
    ----------
    nPhi : list
        Next angular velocity error between internal and measurement.
    oPhi : ndarray
        Prior angular velocity error between internal and measurement.
    time_step : float
        Time step.
    weight_kin : float
        Kinematic term weights.
    weight_pot : float
        Potential term weights.
    mDamp : ndarray
        Observer damping parameter.
    nSenAnv : ndarray
        Next angular velocity measurement.
    nPotErr : [type]
        Next attitude measurements error.

    Returns
    -------
    list
        Residual value.

    Side Effects
    ------------
    1. Call exponential matrix from scipy.

    Restrictions
    ------------
    1. nPhi must be list, contrary to regular use.

    '''
    # format new phi into an column vector array
    vecNewPhi = np.array([[nPhi[0]], [nPhi[1]], [nPhi[2]]])
    # compute residual
    fVal =  (
        - weight_kin * vecNewPhi 
        + expm(-time_step * skew_sym( nSenAnv - vecNewPhi )) @ ( 
            weight_kin * oPhi 
            - time_step * mDamp @ oPhi 
            - time_step * weight_pot * inv_skew_sym(nPotErr) 
        )
    )
    # get residual as list
    res = fVal.T.tolist()[0]

    return res


def next_atd_estimate(timeStep, oldAtd, oldAnv):
    '''
    Return next attitude estimate from discrete kinematics.

    Parameters
    ----------
    timeStep : float
        Time step.
    oldAtd : ndarray
        Prior atitude estimate matrix.
    oldAnv : ndarray
        Prior angular velocity estimate vector.

    Returns
    -------
    ndarray
        Next attitude estimate matrix.

    '''
    newAtd = oldAtd @ expm( timeStep * skew_sym(oldAnv) )
    return newAtd


def next_obs_feedback(oPhi, time_step, weight_kin, weight_pot, mDamp, nSenAnv, nPotErr):
    '''
    Return next observer feedback from its dynamics residual function.

    Parameters
    ----------
    oPhi : ndarray
        Prior angular velocity error between internal and measurement.
    time_step : float
        Time step.
    weight_kin : float
        Kinematic term weights.
    weight_pot : float
        Potential term weights.
    mDamp : ndarray
        Observer damping parameter.
    nSenAnv : ndarray
        Next angular velocity measurement.
    nPotErr : [type]
        Next attitude measurements error.

    Returns
    -------
    ndarray
        Next observer feedback.

    Raises
    ------
    ValueError
        Unsuccessful computation of residual zero.

    '''
    # collect observer arguments
    mArgs = (oPhi, time_step, weight_kin, weight_pot, mDamp, nSenAnv, nPotErr)
    # compute zero of residual function
    nPhi = opt.root(anv_observer, oPhi, args=mArgs, method='hybr')

    if nPhi.success:
        return np.array([nPhi.x]).T
    else:
        raise ValueError


def compute_measured_rotation(mea_data, obs_state, obs_par):
    ''' Compute rotation matrix from measurement data.

        Parameters
        ----------
        newMea
            Next measurement data.

        Returns
        -------
        ndarray
            New attitude matrices.
    '''
    mea_atd, _ = compute_solution(mea_data, obs_state, obs_par)

    return mea_atd[kpkg.ObsModelKeys.atd]


# def compute_measured_rotation_with_cov(newMea : Sensors, **kwargs):
#     '''
#     Compute rotation matrix from measurement data.

#     Parameters
#     ----------
#     newMea
#         Next measurement data.

#     Returns
#     -------
#     ndarray
#         New attitude matrices.
#     '''
#     # get measurements
#     mArg = {kpkg.ObsKeys.KEY_MEAS : newMea}

#     kwargs.update(**mArg)
#     # compute deterministic solution
#     senAtd, covar, _ = compute_solution(**kwargs)
#     # return attitude data
#     return senAtd, covar


def variational_estimator_dsc_ind(
    mea_data : dict[str, DataPoint],
    obs_state : dict[str, DataPoint],
    obs_par : dict
) -> Tuple[dict[str, DataPoint], dict[str, DataPoint]]:
    '''
    Return estimated attitude and observer internal state.

    It follows the discrete variational estimator.

    Returns
    -------
    dict
        Observer estimates with key from manager.

    Side Effects
    ------------
    1. Computes deterministic attitude.
    2. Calls exponential matrix method from scipy.
    3. Calls root method from scipy.

    Restrictions
    ------------
    1. Deterministic attiude must exist.

    '''
    # get parameters
    # get current time
    itime : float = mea_data[kpkg.kpro.SenModelKeys.anv].time

    # get time step key and value
    time_step : float = obs_par[kpkg.ObsModelKeys.time_step]

    # get prior observer state
    old_est_atd = obs_state[kpkg.ObsModelKeys.atd]
    old_est_anv = obs_state[kpkg.ObsModelKeys.anv]
    old_est_phi = obs_state[kpkg.ObsModelKeys.anv_err]

    # TODO : simpler
    # TODO : build class
    # get observer parameters
    m_damp = {}
    for index, damp_list in obs_par[kpkg.ObsModelKeys.obs_damp].data.items():
        for idamp in damp_list:
            if idamp[0] <= itime:
                m_damp[index] = np.array(idamp[1])

    w_pot = {}
    for index, ipot_list in obs_par[kpkg.ObsModelKeys.weight_pot].data.items():
        for ipot in ipot_list:
            if ipot[0] <= itime:
                w_pot[index] = np.array(ipot[1])

    w_kin = {}
    for index, ikin_list in obs_par[kpkg.ObsModelKeys.weight_kin].data.items():
        for ikin in ikin_list:
            if ikin[0] <= itime:
                w_kin[index] = np.array(ikin[1])

    # get next measurement values
    new_mea_anv : DataPoint = mea_data[kpkg.kpro.SenModelKeys.anv]

    # compute new rotation from measurements
    new_mea_atd : AttitudeData = compute_measured_rotation(mea_data, {}, obs_par)

    # estimate attitude
    new_atd_data = {}
    for index in [(0,1), (0,2), (0,3)]:
        idx_anv = (index[1], index[0])
        new_atd_data[index] = orthogonalize_mat(
            get_next_atd_dis(
                time_step,
                old_est_atd.data[index],
                old_est_anv.data[idx_anv]
            )
        )

    # save attitude data
    new_obs_atd = AttitudeData(new_mea_anv.time)
    new_obs_atd.setup_data(new_atd_data)

    # estimate angular velocity
    new_phi_data = {}
    new_obs_anv_data = {}
    for index in [(0,1), (0,2), (0,3)]:
        anv_index = (index[1], index[0])

        new_lmat = (new_obs_atd.data[index].T @ new_mea_atd.data[index]) - (new_obs_atd.data[index].T @ new_mea_atd.data[index]).T

        new_phi_data[anv_index] = next_obs_feedback(
            old_est_phi.data[anv_index],
            time_step,
            w_kin[anv_index],
            w_pot[index],
            m_damp[anv_index],
            new_mea_anv.data[anv_index],
            new_lmat
        )

        new_obs_anv_data[anv_index] = new_mea_anv.data[anv_index] - new_phi_data[anv_index]


    # save angular velocity
    new_obs_phi = AngularVelocityData(new_mea_anv.time)
    new_obs_phi.setup_data(new_phi_data)

    new_obs_anv = AngularVelocityData(new_mea_anv.time)
    new_obs_anv.setup_data(new_obs_anv_data)

    # save to output dictionary
    estimates = {
        kpkg.ObsKeys.atd : new_obs_atd,
        kpkg.ObsKeys.anv : new_obs_anv,
        kpkg.ObsKeys.anv_err : new_obs_phi
    }

    return estimates, {}


# TODO : adapt to new structure
# def variational_estimator_weights(**kwargs):
#     '''
#     Return estimated attitude and observer internal state.

#     It follows the discrete variational estimator.

#     Returns
#     -------
#     dict
#         Observer estimates with key from manager.

#     Side Effects
#     ------------
#     1. Computes deterministic attitude.
#     2. Calls exponential matrix method from scipy.
#     3. Calls root method from scipy.

#     Restrictions
#     ------------
#     1. Deterministic attiude must exist.

#     '''
#     # get parameters
#     time_step : float = kwargs[kpkg.SettKeys.OPT_TIME_STEP]
#     params : dict = kwargs[kpkg.ObsKeys.KEY_PAR]
#     newMea : dict = kwargs[kpkg.ObsKeys.KEY_MEAS]
#     oldEst : dict = kwargs[kpkg.ObsKeys.KEY_OLD]

#     # get covariance
#     # kwargs[]

#     # get prior observer state
#     old_est_atd = oldEst[kpkg.ObsKeys.atd]
#     old_est_anv = oldEst[kpkg.ObsKeys.anv]
#     old_est_phi = oldEst[kpkg.ObsKeys.anv_err]

#     # get observer parameters
#     matDamping = params[kpkg.ObsKeys.obs_damp]
#     potWeightsin = params[kpkg.ObsKeys.weight_pot]
#     kinWeightsin = params[kpkg.ObsKeys.weight_kin]

#     # TODO : get params from list
#     kinWeights = np.array(kinWeightsin.data[0][1], float)
#     potWeights = np.array(potWeightsin.data[0][1], float)
#     auxDmp = np.empty((4,4), object)
#     for i in [1,2,3]:
#         auxDmp[i,i] = np.array(matDamping.data[i-1][2])
#     matDamping = auxDmp

#     # get new measurements
#     newSenAnv = newMea[kpkg.SenKeys.anv]

#     # compute new rotation from measurements
#     new_mea_atd, newAtdCov  = compute_measured_rotation_with_cov(newMea, **kwargs)
#     new_mea_atd : AttitudeData
#     newSenRi1 = new_mea_atd.data[(0,1)]
#     newSenRi2 = new_mea_atd.data[(0,2)]
#     newSenRi3 = new_mea_atd.data[(0,3)]

#     # compute next rotation estimate
#     newRi1 = next_atd_estimate(time_step, old_est_atd.data[0,1], old_est_anv.data[1,0])
#     newRi2 = next_atd_estimate(time_step, old_est_atd.data[0,2], old_est_anv.data[2,0])
#     newRi3 = next_atd_estimate(time_step, old_est_atd.data[0,3], old_est_anv.data[3,0])

#     # format estimate as attitude data
#     tmp = np.empty((4,4), object)
#     tmp[0,1] = newRi1
#     tmp[0,2] = newRi2
#     tmp[0,3] = newRi3
#     newEstAtd = AttitudeData(newSenAnv.time)
#     newEstAtd.setup_data(tmp)

#     # compute skew potential errors
#     newLi1 = (newRi1.T @ newSenRi1 - newSenRi1.T @ newRi1)
#     newLi2 = (newRi2.T @ newSenRi2 - newSenRi2.T @ newRi2)
#     newLi3 = (newRi3.T @ newSenRi3 - newSenRi3.T @ newRi3)

#     # NOTE New weights
#     sen_sdev = 260*10**-5
#     sen_wght = 1

#     # for idx in [1, 2, 3]:
#     #     new_wght = sen_wght - (sen_wght - 0.01 * sen_wght) * (sen_sdev - np.linalg.norm(newAtdCov.data[(0, idx)])) / (sen_sdev - 100 * sen_sdev)
#     #     if new_wght < 0 or np.isnan(new_wght):
#     #         new_wght = 0#10**-2
#     #         # print('force 0')
#     #     potWeights[(idx,idx)] = new_wght

#     limit = [0, 2.79 * sen_sdev, 7.55 * sen_sdev, 7.09 * sen_sdev]
#     for idx in [1, 2, 3]:
#         if np.linalg.norm(newAtdCov.data[(0, idx)]) > limit[idx]:
#             potWeights[(idx,idx)] = 10**-3
#         if np.isnan(np.linalg.norm(newAtdCov.data[(0, idx)])) :
#             potWeights[(idx,idx)] = 10**-6

#     # print(potWeights[(1,1)], potWeights[(2,2)], potWeights[(3,3)])

#     # compute next angular velocity error estimate
#     newPhi1 = next_obs_feedback(old_est_phi.data[(1,0)],
#                                 time_step,
#                                 kinWeights[(1,1)],
#                                 potWeights[(1,1)],
#                                 matDamping[(1,1)],
#                                 newSenAnv.data[(1,0)],
#                                 newLi1)
#     newPhi2 = next_obs_feedback(old_est_phi.data[(2,0)],
#                                 time_step,
#                                 kinWeights[(2,2)],
#                                 potWeights[(2,2)],
#                                 matDamping[(2,2)],
#                                 newSenAnv.data[(2,0)],
#                                 newLi2)
#     newPhi3 = next_obs_feedback(old_est_phi.data[(3,0)],
#                                 time_step,
#                                 kinWeights[(3,3)],
#                                 potWeights[(3,3)],
#                                 matDamping[(3,3)],
#                                 newSenAnv.data[(3,0)],
#                                 newLi3)

#     # compute next observer internal angular velocity
#     newAnv1 = newSenAnv.data[(1,0)] - newPhi1
#     newAnv2 = newSenAnv.data[(2,0)] - newPhi2
#     newAnv3 = newSenAnv.data[(3,0)] - newPhi3

#     # format into ndarray
#     tmp2 = np.empty((4,4), object)
#     tmp2[1,0] = newPhi1
#     tmp2[2,0] = newPhi2
#     tmp2[3,0] = newPhi3

#     tmp3 = np.empty((4,4), object)
#     tmp3[1,0] = newAnv1
#     tmp3[2,0] = newAnv2
#     tmp3[3,0] = newAnv3

#     # format as angular velocity data
#     newPhiAnv = AngularVelocityData(newSenAnv.time)
#     newPhiAnv.setup_data(tmp2)
#     newEstAnv = AngularVelocityData(newSenAnv.time)
#     newEstAnv.setup_data(tmp3)

#     # add observer data to output
#     estimateDict = {
#         kpkg.ObsKeys.atd : newEstAtd,
#         kpkg.ObsKeys.anv : newEstAnv,
#         kpkg.ObsKeys.anv_err : newPhiAnv,
#         kpkg.ObsKeys.atd_sen : new_mea_atd
#     }

#     return estimateDict
