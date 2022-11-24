#!/usr/bin/env python3
''' Functions for the deterministic solution.

    Considers the attitude problem of a heterogeneous three-vehicle formation.

'''
from typing import Tuple
# import third-party
import numpy as np

# import from package
from for_atd_sim.lib_data.basic_data import DataPoint
from for_atd_sim.lib_data.centralized_data import AttitudeData
from for_atd_sim.lib_math.so3 import skew_sym
from for_atd_sim.lib_math.so3 import rot_mtx
from for_atd_sim.lib_math import interval_pm_pi

# import from subpackage
import for_atd_sim.obs_detatd_inerel.keys as kpkg
from for_atd_sim.obs_detatd_inerel.covariance import CovarAtd
from for_atd_sim.obs_detatd_inerel.structures import CoefficientArray
from for_atd_sim.obs_detatd_inerel.structures import InertiaslSolution
from for_atd_sim.obs_detatd_inerel.structures import RelativeCandidate
from for_atd_sim.obs_detatd_inerel.structures import RelativeSolution

kPhiAC = 'phiac_observer_value'
kPhiAD = 'phiad_observer_value'
kPhiBC = 'phibc_observer_value'
kPhiBD = 'phibd_observer_value'

kAlphas = 'configuration_alphas'


def valid_measurement(*vecList) -> bool:
    ''' Test if measurements exist and are unit vectors.

        Returns
        -------
        bool
            False if it fails test.
    '''
    # set tolerance
    tol = 10**-4

    for vec_i in vecList:
        # test existance
        if vec_i is None:
            return False
        # test unit norm
        norm_i = np.linalg.norm(vec_i)
        if norm_i > 1 + tol or norm_i < 1 - tol:
            return False

    return True


def compute_solution(
    mea_data : dict[str, DataPoint], 
    obs_state : dict[str, DataPoint], 
    obs_par : dict
) -> Tuple[dict[str, DataPoint], dict[str, DataPoint]]:
    ''' Return attitude for a three vehicle heterogeneous formation.

        Arguments
        ---------
        **kwargs
            Dictionary with the measurements taken.

        Raises
        ------
        AssertionError
            Measurements not valid.

        Returns
        -------
        dict
            Estimated attitude.

    '''
    extra_data = {}

    covar = CovarAtd(mea_data)

    # parse options
    opt_covar = obs_par.get(kpkg.OPT_COVARIANCE, True) 
    opt_optimize = obs_par.get(kpkg.OPT_PROCRUSTES, True)

    # parse data structures
    sen_ref : DataPoint = mea_data[kpkg.SenModelKeys.ref]
    sen_los : DataPoint = mea_data[kpkg.SenModelKeys.los]

    # parse time
    time = float(sen_ref.time) # FIXME should be float alre

    # parse measurements
    d1 = sen_ref.data[(1,1)]
    d2 = sen_ref.data[(2,2)]
    d3 = sen_ref.data[(3,3)]
    di1 = sen_ref.data[(1,0)]
    di2 = sen_ref.data[(2,0)]
    di3 = sen_ref.data[(3,0)]

    d12 = sen_los.data[(1,2)]
    d21 = sen_los.data[(2,1)]
    d13 = sen_los.data[(1,3)]
    d31 = sen_los.data[(3,1)]

    # validate measurements
    if not valid_measurement(d1, d2, d3, d12, d21, d13, d31, di1, di2, di3):
        raise AssertionError

    # get candidates for R12
    drel12 = relative_solution(d12, d21, d1, di1, d2, di2)

    # get candidates for R13
    drel13 = relative_solution(d13, d31, d1, di1, d3, di3)

    # TODO adapt relative solution to have general constraint sign
    # get candidates for RI1A
    ine_1a = relative_solution(-di1, d1, di2, d2, drel12.can_a.rot@d2, d2)

    ine_1b = relative_solution(-di1, d1, di2, d2, drel12.can_b.rot@d2, d2)

    ine_1c = relative_solution(-di1, d1, di3, d3, drel13.can_a.rot@d3, d3)

    ine_1d = relative_solution(-di1, d1, di3, d3, drel13.can_b.rot@d3, d3)

    # compare and disambiguate
    ine_sol = compare_rotations(
        ine_1a.can_a, ine_1b.can_a, ine_1c.can_a, ine_1d.can_a,
        drel12.can_a, drel12.can_b, drel13.can_a, drel13.can_b,
        time
    )

    # compute branch covariance
    # TODO : test covariance option
    if opt_covar:
        # branch covariance        
        covar.set_branch_solution(ine_sol)
        covar.compute_covariance_branches()
        
        if opt_optimize:
            ine_sol.rot_i1.rot, weight_list = average_weighted_rotation(
                ine_sol.rot_x.rot, ine_sol.rot_y.rot,
                covar.exp_rotine_brn_list[2], covar.exp_rotine_brn_list[3]
            )

            # inertial covariance
            covar.atd_data.data[(0,1)] = ine_sol.rot_i1.rot
            covar.compute_covariance_inertial(weight_list) #FIXME requires procrustes

            covar.compute_other_covariance()
            
        else:
            # average rotation
            ine_sol.rot_i1.rot = average_rotation(ine_sol.rot_x.rot, ine_sol.rot_y.rot)

            # TODO inertial covariance
            covar.atd_data.data[(0,1)] = ine_sol.rot_i1.rot
            raise AssertionError
            # TODO
            # covar.compute_covariance_inertial(weight_list) #FIXME requires procrustes

    elif opt_optimize:
        print('Covariance required for Procrustes.')
        raise AssertionError

    else:
        # average rotation
        ine_sol.rot_i1.rot = average_rotation(ine_sol.rot_x.rot, ine_sol.rot_y.rot)

    # compute the remaining rotations
    ine_sol.rot_23.rot = np.transpose(ine_sol.rot_12.rot) @ ine_sol.rot_13.rot
    ine_sol.rot_i2.rot = ine_sol.rot_i1.rot @  ine_sol.rot_12.rot
    ine_sol.rot_i3.rot = ine_sol.rot_i1.rot @  ine_sol.rot_13.rot

    # format attitude estimate
    atd_est = AttitudeData()
    atd_est.time=time

    atd_est.data[(0,1)] = ine_sol.rot_i1.rot
    atd_est.data[(0,2)] = ine_sol.rot_i2.rot
    atd_est.data[(0,3)] = ine_sol.rot_i3.rot

    atd_est.complete_data()

    # get and save alpha relation
    alpha1 = float(compute_alpha(di1, di2, di3))
    alpha2 = float(compute_alpha(d1, d12, d13))
    extra_data[kAlphas] = DataPoint(time)
    extra_data[kAlphas].data[(0,0)] = np.array([[np.cos(alpha1-alpha2)], [alpha1], [alpha2]])

    extra_data.update({**ine_sol.comparison_scores})
    extra_data.update({kpkg.ObsKeys.atd_cov : covar.cov_pnt})
    extra_data.update({kpkg.ObsKeys.atd_sdv : covar.sdev_pnt})

    return {kpkg.ObsKeys.atd : atd_est}, extra_data


def relative_solution(d12, d21, d1, i1, d2, i2, *keys) -> RelativeCandidate:
    ''' Return relative attitude candidates between the chief and a deputy.

        Arguments
        ---------
        d12 : numpy.ndarray
            LOS from chief to deputy
        d21 : numpy.ndarray
            LOS from deputy to chief
        d1  : numpy.ndarray
            Inertial of the chief in body
        i1  : numpy.ndarray
            Inertial of the chief in inertial
        d2  : numpy.ndarray
            Inertial of the deputy in body
        i2  : numpy.ndarray
            Inertial of the deputy in inertial

        Returns
        -------
        dict
            Solutions and extra variables as {R12A R12B {aux_rota aux_rotb}}

    '''
    rel_can = RelativeCandidate()

    # test if sum of axis is null vector
    if np.linalg.norm(d21-d12) != 0:
        rot_1 = rot_mtx(np.pi,(d21-d12) / np.linalg.norm(d21-d12))
    else:
        rot_1 = rot_mtx(
            np.pi,skew_sym(d1) @ d12 / np.linalg.norm(skew_sym(d1) @ d12)
        )

    # compute the trigonometric coeffiecients
    my_coef = CoefficientArray(d1, i1, d2, i2, d12, rot_1)

    # TODO when approx -1 but out
    if abs(my_coef.cp / np.sqrt(my_coef.cs**2 + my_coef.cc**2)) <= 1:
        theta2a = (
            + np.arctan2(my_coef.cs, my_coef.cc)
            + np.arccos (my_coef.cp / np.sqrt(my_coef.cs**2 + my_coef.cc**2))
        )
        theta2b = (
            + np.arctan2(my_coef.cs, my_coef.cc)
            - np.arccos (my_coef.cp / np.sqrt(my_coef.cs**2 + my_coef.cc**2))
        )
    else:
        # BUG return
        theta2a = (
            + np.arctan2(my_coef.cs, my_coef.cc)
            + np.arccos (np.sign(my_coef.cp / np.sqrt(my_coef.cs**2 + my_coef.cc**2)))
        )
        theta2b = (
            + np.arctan2(my_coef.cs, my_coef.cc)
            - np.arccos (np.sign(my_coef.cp / np.sqrt(my_coef.cs**2 + my_coef.cc**2)))
        )

    # put values of the angles in the interval (-pi,pi]
    theta2a = interval_pm_pi(theta2a)
    theta2b = interval_pm_pi(theta2b)

    # compute second rotation
    rot_2a = rot_mtx(theta2a, -d12)
    rot_2b = rot_mtx(theta2b, -d12)

    # compute both solutions
    rel_can.can_a.rot = rot_2a @ rot_1
    rel_can.can_b.rot = rot_2b @ rot_1

    # add auxiliary variables do dict
    rel_can.can_a.aux.rot_1 = rot_1
    rel_can.can_a.aux.rot_2 = rot_2a
    rel_can.can_a.aux.coef = my_coef
    rel_can.can_a.aux.beta = theta2a[0,0]
    rel_can.can_a.aux.sign = +1
    
    rel_can.can_b.aux.rot_1 = rot_1
    rel_can.can_b.aux.rot_2 = rot_2b
    rel_can.can_b.aux.coef = my_coef
    rel_can.can_b.aux.beta = theta2b[0,0]
    rel_can.can_b.aux.sign = -1

    return rel_can


def triad_algorithm(d1, i1, rot_d2, i2) -> np.ndarray:
    ''' Return optimal attitude matrix.

        Implement TRIAD algorithm.

        Arguments
        ---------
        d1  : numpy.ndarray
            Inertial of the chief in body
        i1  : numpy.ndarray
            Inertial of the chief in inertial
        rot_d2 : numpy.ndarray
            Inertial of the deputy in chief's body
        i2  : numpy.ndarray
            Inertial of the deputy in inertial

        Returns
        -------
        np.ndarray
            Optimal rotation matrix.

    '''
    # test if sum of axis is null vector
    if np.linalg.norm(d1-rot_d2) == 0 or np.linalg.norm(i1-i2) == 0:
        print("TRIAD not valid")

    # auxiliary Matrices
    auxi = (skew_sym(i1) @ i2) / (np.linalg.norm(skew_sym(i1) @ i2))
    aux1 = (skew_sym(d1) @ rot_d2) / (np.linalg.norm(skew_sym(d1) @ rot_d2))

    # inertial Attitude Computation
    rot_i1 = (
        + i1 @ np.transpose(d1)
        + auxi @ np.transpose(aux1)
        + (skew_sym(i1) @ auxi) @ np.transpose(skew_sym(d1) @ aux1)
    )

    return rot_i1

# TODO : get principal angle
def evaluate_proximity(rot1, rot2):
    ''' Return angle between two approximate rotations. '''
    proximity = abs(3 - np.trace(rot1 @ np.transpose(rot2)))
    return proximity


def compare_rotations(
    ine_a : RelativeSolution, ine_b : RelativeSolution, 
    ine_c : RelativeSolution, ine_d : RelativeSolution, 
    rel_a : RelativeSolution, rel_b : RelativeSolution, 
    rel_c : RelativeSolution, rel_d : RelativeSolution, 
    time
) -> InertiaslSolution:
    ''' Compare different candidates and select solution.

        Arguments
        ---------
        **kwargs
            Dictionary with data and keys.

        Returns
        -------
        dict
            Solutions, candidates, and extra variables.
    '''
    # Initialize Array
    proximity_val = np.empty((4,1))
    proximity_val2 = np.empty((4,1))
    # my_cov = np.empty((4,4), object)

    # inertial comparison set
    ine_set = np.array(
        [[ine_a, ine_c],
        [ine_a, ine_d],
        [ine_b, ine_c],
        [ine_b, ine_d]]
    )

    rel_set = np.array(
        [[rel_a, rel_c],
        [rel_a, rel_d],
        [rel_b, rel_c],
        [rel_b, rel_d]]
    )

    # get the comparison values
    for i in range(4):
        proximity_val[i] = evaluate_proximity(ine_set[i][0].rot, ine_set[i][1].rot)
        proximity_val2[i] = np.arccos((np.trace(ine_set[i][0].rot @ ine_set[i][1].rot.T) - 1)/2)

    # find the index for the minimum
    mindex = np.argmin(proximity_val)

    # Parse solutions for readability
    sol_rot12 = rel_set[mindex][0]
    sol_rot13 = rel_set[mindex][1]

    # get inertial rotations
    sol_roti1_br2 = ine_set[mindex][0]
    sol_roti1_br3 = ine_set[mindex][1]

    # build output 
    ine_sol = InertiaslSolution()
    ine_sol.rot_12 = sol_rot12
    ine_sol.rot_13 = sol_rot13
    ine_sol.rot_x = sol_roti1_br2
    ine_sol.rot_y = sol_roti1_br3

    ac_dp = DataPoint(time)
    ad_dp = DataPoint(time)
    bc_dp = DataPoint(time)
    bd_dp = DataPoint(time)

    ac_dp.data[(0,0)] = np.array([[[proximity_val2[0]]]])
    ad_dp.data[(0,0)] = np.array([[[proximity_val2[1]]]])
    bc_dp.data[(0,0)] = np.array([[[proximity_val2[2]]]])
    bd_dp.data[(0,0)] = np.array([[[proximity_val2[3]]]])

    ine_sol.comparison_scores.update({kPhiAC : ac_dp})
    ine_sol.comparison_scores.update({kPhiAD : ad_dp})
    ine_sol.comparison_scores.update({kPhiBC : bc_dp})
    ine_sol.comparison_scores.update({kPhiBD : bd_dp})

    return ine_sol


def average_rotation(rot1, rot2) -> np.ndarray:
    ''' Return the solution to the orthogonal Procrustes problem.

        Parameters
        ----------
        rot1 : numpy.ndarray
            Rotation matrix.
        rot2 : numpy.ndarray
            Rotation matrix.

        Returns
        -------
        np.ndarray
            Average rotation.
    '''
    # create observation matrix
    mat_x = np.concatenate([np.eye(3), np.eye(3)], axis=1)
    mat_y = np.concatenate([rot1, rot2 ], axis=1)

    obs_mat = mat_y @ mat_x.T

    # decompose observation matrix
    svd_u, _, svd_vh = np.linalg.svd(obs_mat)

    # compute average rotation
    opt_rot = svd_u @ np.diag([1,1, np.linalg.det(svd_u @ svd_vh)]) @ svd_vh

    return opt_rot


def average_weighted_rotation(rot1, rot2, cov1_list, cov2_list) -> np.ndarray:
    ''' Return the solution to the orthogonal Procrustes problem.

        Parameters
        ----------
        rot1 : numpy.ndarray
            Rotation matrix.
        rot2 : numpy.ndarray
            Rotation matrix.

        Returns
        -------
        np.ndarray
            Average rotation.
    '''
    mat_x = np.concatenate([np.eye(3), np.eye(3)], axis=1)
    mat_y = np.concatenate([rot1, rot2 ], axis=1)

    weight_list = []
    for cov_1_i in cov1_list:
        if np.isnan(cov_1_i).any():
            cov_1_i = np.eye(3) * 1000 # FIXME value should be large
        weight_list.append(max(np.linalg.eigvals(cov_1_i)))

    for cov_2_i in cov2_list:
        if np.isnan(cov_2_i).any():
            cov_2_i = np.eye(3) * 1000 # FIXME value should be large
        weight_list.append(max(np.linalg.eigvals(cov_2_i)))

    weight_mat = np.diag(weight_list)
    if abs(np.linalg.det(weight_mat)) < 10**-9:
        weight_mat = np.eye(weight_mat.shape[0])

    n_weight = np.ones((6,1)).T @ np.linalg.inv(weight_mat) @ np.ones((6,1))
    mat_w = (
        np.linalg.inv(weight_mat)
        - (
            np.linalg.inv(weight_mat) @ np.ones((6,1))
            @ np.ones((6,1)).T @ np.linalg.inv(weight_mat)
            / n_weight
        )
    )

    # create observation matrix
    obs_mat = mat_y @ mat_w @ mat_x.T #/ n_weight

    # decompose observation matrix
    svd_u, _, svd_vh = np.linalg.svd(obs_mat)

    # compute average rotation
    opt_rot = svd_u @ np.diag([1,1, np.linalg.det(svd_u @ svd_vh)]) @ svd_vh

    return opt_rot, weight_list


def compute_alpha(i1, i2, i3):
    ''' Return angle of ambiguous parameter. '''
    # compute normals
    n3 = skew_sym(i1) @ i3 / np.linalg.norm(skew_sym(i1) @ i3)

    # compute beta
    bs = i1.T @ skew_sym(n3) @ i3
    bc = i1.T @ skew_sym(n3) @ skew_sym(n3) @ i3
    bp = i1.T @ i2

    beta = np.arctan2(bs,bc) + np.arccos(bp / np.sqrt(bs**2 + bc**2))

    # compute alpha
    ib3 = rot_mtx(beta, n3) @ i3

    cs = i2.T @ skew_sym(i1) @ ib3
    cc = i2.T @ skew_sym(i1) @ skew_sym(i1) @ ib3

    # cp = i2.T @ i1 @ i1.T @ ib3 - 1
    # alpha = np.arctan2(cs,cc) + np.arccos(cp / np.sqrt(cs**2 + cc**2))
    alpha = np.arctan2(cs,cc) + np.pi

    return interval_pm_pi(alpha)
