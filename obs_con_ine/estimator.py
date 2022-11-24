''' Discretized attitude variational observer implementation.

    Functions
    ---------
    constrained_var_estimator
        Attitude estimator based on the Lagrange DAlembert principle.
    anv_observer_res
        Residual of discretized observer internal angular velocity dynamics.
    next_obs_feedback
        Return next observer feedback from its dynamics residual function.
    direction_potential_single
        Return potential term of vector direction constraint.
    angle_potential_single
        Return potential term of vector angle constraint.
    direction_potential_double
        Return potential term of vector direction constraint with two rotations.
    angle_potential_double
        Return potential term of vector angle constraint.
    compute_potential
        Return potential observer terms array.

'''
import copy
import time

# import - math and scientific libraries
import numpy as np
import scipy.optimize as opt
from scipy.linalg import expm
from scipy.integrate import solve_ivp
from for_atd_sim.lib_math.so3 import skew_sym

# import - data structures
from for_atd_sim.lib_data.basic_data import DataPoint
from for_atd_sim.lib_data.basic_data import DataSeries
from for_atd_sim.lib_data.centralized_data import AttitudeData
from for_atd_sim.lib_data.centralized_data import AngularVelocityData

# import - models
from for_atd_sim.lib_atd.rigid_body import get_next_atd_dis

# import - subpackage related
import for_atd_sim.obs_con_ine.keys as kpkg
import for_atd_sim.core.observer as obs
import for_atd_sim.lib_data as dlib
from typing import Tuple


# def constrained_var_estimator(**kwargs):
def constrained_var_estimator(    
    mea_data : dict[str, DataPoint], 
    obs_state : dict[str, DataPoint], 
    obs_par : dict
) -> Tuple[dict[str, DataPoint], dict[str, DataPoint]]:
    ''' Return the next observer internal state.

        Based on the constrained attitude variational observer.

        Returns
        -------
        dict
            Observer internal state and debug parameters.

    '''
    # get current time
    itime : float = mea_data[kpkg.kpro.SenModelKeys.anv].time

    # get time step key and value
    time_step : float = obs_par[kpkg.ObsModelKeys.time_step]

    # get prior observer state
    old_est_atd = obs_state[kpkg.ObsModelKeys.atd]
    old_est_anv = obs_state[kpkg.ObsModelKeys.anv]
    old_est_phi = obs_state[kpkg.ObsModelKeys.anv_err]

    # get observer parameters
    # TODO : simpler
    mat_damping = {}
    for index in obs_par[kpkg.ObsModelKeys.obs_damp].data:
        for idamp in obs_par[kpkg.ObsModelKeys.obs_damp].data[index]:
            if idamp[0] <= itime:
                mat_damping[index] = np.array(idamp[1])

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


    # for ipot in obs_par[kpkg.ObsModelKeys.weight_pot].data[(0,0)]:
    #     if ipot[0] <= itime:
    #         w_pot = np.array(ipot[1])

    # for ikin in obs_par[kpkg.ObsModelKeys.weight_kin].data[(0,0)]:
    #     if ikin[0] <= itime:
    #         w_kin = np.array(ikin[1])

    # get next measurement values
    new_mea_anv : DataPoint = mea_data[kpkg.kpro.SenModelKeys.anv]
    new_mea_ref : DataPoint = mea_data[kpkg.kpro.SenModelKeys.ref]
    new_mea_los : DataPoint = mea_data[kpkg.kpro.SenModelKeys.los]

    # estimate attitude
    new_atd_data = {}
    for index in [(0,1), (0,2), (0,3)]:
        idx_anv = (index[1], index[0])
        new_atd_data[index] = get_next_atd_dis(
            time_step,
            old_est_atd.data[index],
            old_est_anv.data[idx_anv]
        )
   
    # save attitude data
    new_obs_atd = AttitudeData(new_mea_anv.time)
    new_obs_atd.setup_data(new_atd_data)

    potArr = compute_potential_var(
        los = new_mea_los.data,
        ref = new_mea_ref.data,
        atd = new_obs_atd.data,
        wpot = w_pot
    )

    # estimate angular velocity
    new_phi_data = {}
    new_obs_anv_data = {}
    for index in [(1,0), (2,0), (3,0)]:
        # compute next observer feedback
        new_phi_data[index] = next_obs_feedback(
            oPhi= old_est_phi.data[index],
            time_step= time_step,
            weight_kin= w_kin[index],
            mDamp= mat_damping[index],
            nSenAnv= new_mea_anv.data[index],
            nPot= potArr[index]
        )
        
        # compute observer internal angular velocities
        new_obs_anv_data[index] = new_mea_anv.data[index] - new_phi_data[index]

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


def continuous_anv_observer(
        t : float,
        nPhi : np.ndarray,
        oPhi : np.ndarray,
        time_step : float,
        weight_kin : float,
        mDamp : np.ndarray,
        nSenAnv : np.ndarray,
        nPot : np.ndarray
    ) -> np.ndarray:
    '''
    Return the angular acceleration as a 1D array.

    Arguments
    ---------
    t : float
        Current time
    anv : np.ndarray
        Angular velocity.
    moi : np.ndarray
        Moment of inertia of the vehicle.
    torque : np.ndarray
        External torque applied to the vehicle.

    Returns
    -------
    np.ndarray
        Angular acceleration.

    Restrictions
    ------------
    1. Angular Velocity input must be 1D list

    '''
    # transform angular velocity into column vector
    vecNewPhi = np.array([[nPhi[0]], [nPhi[1]], [nPhi[2]]])

    # compute angular acceleration
    vecNewPhi = 1 / weight_kin * (nPot - (weight_kin * skew_sym(nSenAnv) + mDamp) @ oPhi )
    # return 1D array
    return vecNewPhi[:,0]


def anv_observer_res(
            nPhi : np.ndarray,
            oPhi : np.ndarray,
            time_step : float,
            weight_kin : float,
            mDamp : np.ndarray,
            nSenAnv : np.ndarray,
            nPot : np.ndarray
) -> np.ndarray:
    ''' Residual of discretized observer internal angular velocity dynamics.

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
        nPot : [type]
            Next attitude error potential term.

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
            - time_step * nPot
            + (weight_kin * np.eye(3) - time_step * mDamp) @ oPhi
        )
    )

    # # # FIXME
    # n1 = np.array([[1],[0],[0]])
    # jac1 = 1 * (
    #     - weight_kin * n1
    #     + (- time_step * skew_sym(n1))
    #     @ expm(- time_step * skew_sym( nSenAnv - vecNewPhi ))
    #     @ ( time_step * nPot + (weight_kin * np.eye(3) - time_step * mDamp) @ oPhi )
    # )

    # n2 = np.array([[0],[1],[0]])
    # jac2 =  1 * (
    #     - weight_kin * n2
    #     + (- time_step * skew_sym(n2))
    #     @ expm(- time_step * skew_sym( nSenAnv - vecNewPhi ))
    #     @ ( time_step * nPot + (weight_kin * np.eye(3) - time_step * mDamp) @ oPhi )
    # )

    # n3 = np.array([[0],[0],[1]])
    # jac3 =  1 * (
    #     - weight_kin * n3
    #     + (- time_step * skew_sym(n3))
    #     @ expm(- time_step * skew_sym( nSenAnv - vecNewPhi ))
    #     @ ( time_step * nPot + (weight_kin * np.eye(3) - time_step * mDamp) @ oPhi )
    # )

    # # # return residual as 1D list
    res = fVal.T.tolist()[0]
    # jac = [
    #     [jac1[0,0], jac2[0,0], jac3[0,0]],
    #     [jac1[1,0], jac2[1,0], jac3[1,0]],
    #     [jac1[2,0], jac2[2,0], jac3[2,0]],
    # ]

    return res#, jac


def next_obs_feedback(
        oPhi : np.ndarray,
        time_step : float,
        weight_kin : float,
        mDamp : np.ndarray,
        nSenAnv : np.ndarray,
        nPot : np.ndarray
        ) -> np.ndarray:
    ''' Return next observer feedback from its dynamics residual function.

        Parameters
        ----------
        oPhi : ndarray
            Prior angular velocity error between internal and measurement.
        time_step : float
            Time step.
        weight_kin : float
            Kinematic term weights.
        mDamp : ndarray
            Observer damping parameter.
        nSenAnv : ndarray
            Next angular velocity measurement.
        nPot : [type]
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
    time_opt = time.time()
    # collect observer arguments
    mArgs = (oPhi, time_step, weight_kin, mDamp, nSenAnv, nPot)

    # compute zero of residual function
    nPhi = opt.root(
        anv_observer_res,
        oPhi,
        args=mArgs,
        method='hybr',
        jac=False
    )
    time_opt = time.time() - time_opt


    # time_opt2 = time.time()
    # nPhi = opt.minimize(
    #     anv_discrete_abs,
    #     oPhi.flatten(),
    #     args=mArgs,
    #     method='nelder-mead',#'BFGS'
    #     # jac=anv_discrete_abs_der
    # )

    # time_opt2 = time.time() - time_opt2

    # return if successful
    if nPhi.success:
        return np.array([nPhi.x]).T
    else:
        raise ValueError


def direction_potential_single(
        refA : np.ndarray,
        matR : np.ndarray,
        vecB : np.ndarray,
        w_pot : float
        ) -> np.ndarray:
    '''
    Return potential term of vector direction constraint.

    Constraint equation : A = R @ B
    Potential equation : (A - R @ B)^T(A - R @ B)

    Parameters
    ----------
    refA : np.ndarray
        Reference vector.
    matR : np.ndarray
        Rotation matrix.
    vecB : np.ndarray
        Body vector.
    w_pot : float
        Constraint weight.

    Returns
    -------
    np.ndarray
        Potential vector term for observer.

    '''
    return - w_pot * skew_sym(vecB) @ matR.T @ refA
    #BUG
    # return w_pot * (refA.T @ matR @ vecB - 1) * skew_sym(vecB) @ matR.T @ refA


def angle_potential_single(
        vecA : np.ndarray,
        matR : np.ndarray,
        vecB : np.ndarray,
        refA : np.ndarray,
        refB : np.ndarray,
        w_pot : float
        ) -> np.ndarray:
    '''
    Return potential term of vector angle constraint.

    Constraint equation : iA.T @ iB = A.T @ R @ B
    Potential equation : (iA.T @ iB - A.T @ R @ B)^2

    Parameters
    ----------
    vecA : np.ndarray
        Body vector A.
    matR : np.ndarray
        Rotation matrix.
    vecB : np.ndarray
        Body vector B.
    refA : np.ndarray
        Reference vector A.
    ReferenceB : np.ndarray
        Reference vector B.
    w_pot : float
        Constraint weight.

    Returns
    -------
    np.ndarray
        Potential vector term for observer.

    '''
    # scalar coeffiecient
    coeficient = w_pot * (refA.T @ refB - vecA.T @ matR @ vecB)
    # vector component
    vecTerm = - skew_sym(vecB) @ matR.T @ vecA

    return coeficient * vecTerm


def direction_potential_double(
        refA : np.ndarray,
        lisR : list,
        vecB : np.ndarray,
        w_pot : float
        ) -> list:
    '''
    Return potential term of vector direction constraint with two rotations.

    Constraint equation : A = R0.T @ R1 @ B
    Potential equation : (A - R0.T @ R1 @ B)^T(A - R0.T @ R1 @ B)

    Parameters
    ----------
    refA : np.ndarray
        Reference vector.
    lisR : list
        List of rotations.
    vecB : np.ndarray
        Body vector.
    w_pot : float
        Constraint weight.

    Returns
    -------
    list
        List of potential vector terms for observer.

    Restrictions
    ------------
    R0 is transposed within the function.
    R0.T @ R1 satisfies the constraint.

    '''
    # initialize term list
    potlist = []

    # compute term for R0
    auxA = lisR[1] @ vecB
    pot0 = direction_potential_single(
        refA= auxA,
        matR= lisR[0],
        vecB= refA,
        w_pot= w_pot)
    potlist.append(pot0)

    # compute term for R1
    auxA1 = lisR[0] @ refA
    pot1 = direction_potential_single(
        refA= auxA1,
        matR= lisR[1],
        vecB= vecB,
        w_pot= w_pot)
    potlist.append(pot1)

    return potlist


def angle_potential_double(
        vecA : np.ndarray,
        lisR : list,
        vecB : np.ndarray,
        refA : np.ndarray,
        refB : np.ndarray,
        w_pot : float
        ) -> list:
    '''
    Return potential term of vector angle constraint.

    Constraint equation : iA.T @ iB = A.T @ R0.T @ R1 @ B
    Potential equation : (iA.T @ iB - A.T @ R0.T @ R1 @ B)^2

    Parameters
    ----------
    vecA : np.ndarray
        Body vector A.
    lisR : np.ndarray
        List of rotations.
    vecB : np.ndarray
        Body vector B.
    refA : np.ndarray
        Reference vector A.
    ReferenceB : np.ndarray
        Reference vector B.
    w_pot : float
        Constraint weight.

    Returns
    -------
    list
        List of potential vector terms for observer.

    '''
    # initialize term list
    potlist = []
    # get constraint rotation
    auxR = lisR[0].T @ lisR[1]
    # compute coefficient
    coef = w_pot * (vecA.T @ auxR @ vecB - refA.T @ refB)
    # compute direction terms
    dirPot = np.empty((2), object)
    dirPot[0] = skew_sym(vecA) @ lisR[0].T @ lisR[1] @ vecB
    dirPot[1] = skew_sym(vecB) @ lisR[1].T @ lisR[0] @ vecA

    # save in list
    for i in range(2):
        potlist.append(coef * dirPot[i])

    return potlist


def compute_potential(
        mLos : np.ndarray,
        mRef : np.ndarray,
        estAtd : np.ndarray,
        w_pot : np.ndarray
        ) -> np.ndarray:
    '''
    Return potential observer terms array.

    Parameters
    ----------
    mLos : np.ndarray
        Next los measurements.
    mRef : np.ndarray
        Next reference measurements.
    estAtd : np.ndarray
        Next estimated attitude.
    w_pot : np.ndarray
        Potential weights matrix.

    Returns
    -------
    np.ndarray
        Array with the three potential vectors.

    '''
    # initialize arrays
    refDir = np.empty((4,1), object)
    losDir = np.empty((4,1), object)
    losAng = np.empty((4,1), object)
    mixAng = np.empty((4,1), object)
    potAll = np.empty((4,1), object)

    # initialize with zeros
    for k in [1,2,3]:
        refDir[k,0] = np.zeros((3,1))
        losDir[k,0] = np.zeros((3,1))
        losAng[k,0] = np.zeros((3,1))
        mixAng[k,0] = np.zeros((3,1))
        potAll[k,0] = np.zeros((3,1))

    # compute reference direction constraints potential
    for j in [1,2,3]:
        refDir[j,0] = direction_potential_single(
            refA= mRef[j,0],
            matR= estAtd[0,j],
            vecB= mRef[j,j],
            w_pot= w_pot[j,j])

    # compute reference direction constraints potential
    for index in [(1,2), (1,3)]:
        # set rotation list
        lisR = [estAtd[0,index[0]], estAtd[0,index[1]]]
        # compute potentials
        auxList = angle_potential_double(
            vecA= mRef[(index[0], index[0])],
            lisR= lisR,
            vecB= mRef[(index[1], index[1])],
            refA= mRef[(index[0],0)],
            refB= mRef[(index[1],0)],
            w_pot= w_pot[index[1],index[0]])
        # add to respective term
        refDir[index[0],0] = refDir[index[0],0] + auxList[0]
        refDir[index[1],0] = refDir[index[1],0] + auxList[1]

    # compute los direction contraints potential
    for index in [(1,2), (1,3)]:
        # set rotation list
        lisR = [estAtd[0,index[0]], estAtd[0,index[1]]]
        # compute potentials
        auxList = direction_potential_double(
            refA= -mLos[index],
            lisR= lisR,
            vecB= mLos[(index[1], index[0])],
            w_pot= w_pot[index])
        # add to respective term
        losDir[index[0],0] = losDir[index[0],0] + auxList[0]
        losDir[index[1],0] = losDir[index[1],0] + auxList[1]

    # compute los angle constraints potential
    index = (2,3)
    lisR = [estAtd[0,index[0]], estAtd[0,index[1]]]
    auxList = angle_potential_double(
        vecA= mLos[index[0],1],
        lisR= lisR,
        vecB= mLos[index[1],1],
        refA= mLos[1,index[0]],
        refB= mLos[1,index[1]],
        w_pot= w_pot[index])
    # add to respective term
    losAng[index[0],0] = losAng[index[0],0] + auxList[0]
    losAng[index[1],0] = losAng[index[1],0] + auxList[1]

    # cross angle constraints
    for j in [2, 3]:
        auxPotj = angle_potential_single(
            vecA= mRef[1,0],
            matR= estAtd[0,j],
            vecB= mLos[j,1],
            refA= mRef[1,1],
            refB= -mLos[1,j],
            w_pot= w_pot[0,j])
        auxPot1 = angle_potential_single(
            vecA= mRef[j,0],
            matR= estAtd[0,1],
            vecB= mLos[1,j],
            refA= mRef[j,j],
            refB= -mLos[j,1],
            w_pot= w_pot[j,0])
        # add to respective term
        mixAng[1,0] = mixAng[1,0] + auxPot1
        mixAng[j,0] = mixAng[j,0] + auxPotj

    # sum all contributions
    for i in [1,2,3]:
        potAll[i,0] = refDir[i,0] + losDir[i,0] + 1*losAng[i,0] + 1*mixAng[i,0]

    return potAll


def compute_potential_var(
        los : np.ndarray,
        ref : np.ndarray,
        atd : np.ndarray,
        wpot : np.ndarray
        ) -> np.ndarray:

    pot_var = {}

    pot_var[(1, 0)] = (
        + wpot[1,1] * skew_sym(ref[1,1]) @ atd[0,1].T @ ref[1,0]
        - wpot[1,2] * skew_sym(los[1,2]) @ atd[0,1].T @ atd[0,2] @ los[2,1]
        - wpot[1,3] * skew_sym(los[1,3]) @ atd[0,1].T @ atd[0,3] @ los[3,1]
        + wpot[2,1] * (
            (
                ref[1,0].T @ ref[2,0]
                - ref[1,1].T @ atd[0,1].T @ atd[0,2] @ ref[2,2]
            )
            * skew_sym(ref[1,1]) @ atd[0,1].T @ atd[0,2] @ ref[2,2]
        )
        + wpot[3,1] * (
            (
                ref[1,0].T @ ref[3,0]
                - ref[1,1].T @ atd[0,1].T @ atd[0,3] @ ref[3,3]
            )
            * skew_sym(ref[1,1]) @ atd[0,1].T @ atd[0,3] @ ref[3,3]
        )
    )

    pot_var[(2, 0)] = (
        + wpot[2,2] * skew_sym(ref[2,2]) @ atd[0,2].T @ ref[2,0]
        - wpot[1,2] * skew_sym(los[2,1]) @ atd[0,2].T @ atd[0,1] @ los[1,2]
        + wpot[2,1] * (
            (
                ref[1,0].T @ ref[2,0]
                - ref[1,1].T @ atd[0,1].T @ atd[0,2] @ ref[2,2]
            )
            * skew_sym(ref[2,2]) @ atd[0,2].T @ atd[0,1] @ ref[1,1]
        )
    )

    pot_var[(3, 0)] = (
        + wpot[3,3] * skew_sym(ref[3,3]) @ atd[0,3].T @ ref[3,0]
        - wpot[1,3] * skew_sym(los[3,1]) @ atd[0,3].T @ atd[0,1] @ los[1,3]
        + wpot[3,1] * (
            (
                ref[1,0].T @ ref[3,0]
                - ref[1,1].T @ atd[0,1].T @ atd[0,3] @ ref[3,3]
            )
            * skew_sym(ref[3,3]) @ atd[0,3].T @ atd[0,1] @ ref[1,1]
        )
    )

    return pot_var



# TODO : verify
def compute_potential_true(
        mLos : np.ndarray,
        mRef : np.ndarray,
        estAtd : np.ndarray,
        w_pot : np.ndarray
        ) -> np.ndarray:

    pot = 0
    for i in [1,2,3]:
        pot += 0.5 * (
            (mRef[i,i] - estAtd[0,i].T @ mRef[i,0]).T
            @ (mRef[i,i] - estAtd[0,i].T @ mRef[i,0])
        )

    for i in [2,3]:
        pot += 0.5 * (
            (mLos[1,i] + estAtd[0,1].T @ estAtd[0,i] @ mLos[i,1]).T
            @ (mLos[1,i] + estAtd[0,1].T @ estAtd[0,i] @ mLos[i,1])
        )

    pot += 0.5 * (
        mLos[1,2].T @ mLos[1,3]
        - mLos[2,1].T @ estAtd[0,2].T @ estAtd[0,3] @ mLos[3,1]
    )**2


    for i in [2,3]:
        pot += 0.5 * (
            mRef[1,1].T @ mLos[1,i]
            + mRef[1,0].T @ estAtd[0,i] @ mLos[i,1]
        )**2
        pot += 0.5 * (
            mRef[i,i].T @ mLos[i,1]
            + mRef[i,0].T @ estAtd[0,1] @ mLos[1,i]
        )**2

    return pot


def compute_potential_variational(
        los : np.ndarray,
        ref : np.ndarray,
        obs_atd : np.ndarray,
        wpot : np.ndarray
        ) -> np.ndarray:
    ''' Return potential first variation. '''

    # initilize potential variation at zero
    pot_var = np.empty(4, object)
    for i in [1,2,3]:
        pot_var[i] = np.zeros((3,1))

    # contribution
    for i in [1,2,3]:
        pot_var[i] += (
            - wpot[i,i] * skew_sym(ref[i,i]) @ obs_atd[0,i].T @ ref[i,0]
        )

    for index in [(1,2), (1,3)]:

        inv_idx = (index[1], index[0])
        pot_var[index[0]] += (
            + wpot[index]
            * skew_sym(los[index]) @ obs_atd[0,index[0]].T @ obs_atd[0,index[1]]
            @ los[inv_idx]
        )

        pot_var[index[1]] += (
            + wpot[index]
            * skew_sym(los[inv_idx])
            @ obs_atd[0,index[1]].T @ obs_atd[0,index[0]]
            @ los[index]
        )

        pot_var[index[0]] += (
            wpot[index[1],0]
            * (
                ref[index[1],index[1]].T @ los[inv_idx]
                + ref[index[1],0].T @ obs_atd[0, index[0]] @ los[index]
            )
            * skew_sym(los[index]) @ obs_atd[0, index[0]].T @ ref[index[1], 0]
        )

        pot_var[index[1]] += (
            wpot[0, index[1]]
            * (
                ref[index[0],index[0]].T @ los[index]
                + ref[index[0],0].T @ obs_atd[0, index[1]] @ los[inv_idx]
            )
            * skew_sym(los[inv_idx]) @ obs_atd[0, index[1]].T @ ref[index[0], 0]
        )


    pot_var[2] += (
        - wpot[2,3]
        * (
            los[1,2].T @ los[1,3]
            - los[2,1].T @ obs_atd[0,2].T @ obs_atd[0,3] @ los[3,1]
        )
        * skew_sym(los[2,1]) @ obs_atd[0,2].T @ obs_atd[0,3] @ los[3,1]
    )

    pot_var[3] += (
        - wpot[2,3]
        * (
            los[1,2].T @ los[1,3]
            - los[2,1].T @ obs_atd[0,2].T @ obs_atd[0,3] @ los[3,1]
        )
        * skew_sym(los[3,1]) @ obs_atd[0,3].T @ obs_atd[0,2] @ los[2,1]
    )

    return pot_var


def state_to_list(x):

    # initialize rotations and angular velocities lists
    oAtd = [0]
    oAnvErr = [0]

    # parse rotations
    for i in [0,9,18]:
        oAtd.append(np.array([x[i:i+9]]).reshape(3,3))
    # parse angular velocities
    for i in [27,30,33]:
        oAnvErr.append(np.array([x[i:i+3]]).reshape(3,1))

    return oAtd, oAnvErr


def filter_continuous(
        t : float,
        x : np.ndarray,
        mLos : np.ndarray,
        mRef : np.ndarray,
        sAnv : np.ndarray,
        obs_damp : np.ndarray,
        weight_kin : np.ndarray,
        weight_pot : np.ndarray,
    ) -> np.ndarray:

    # parse variables
    oAtd, oAnvErr = state_to_list(x)

    # get observer parameters
    damp_arr = np.empty((4,4), object)
    for idamp in obs_damp:
        if idamp[0] <= t:
            damp_arr[idamp[1]] = np.array(idamp[2])

    # initialize derivatives
    dx = []
    dAtd = [0]
    dAnvErr = [0]

    arrAtd = np.empty((4,4), object)
    for i in [1,2,3]:
        arrAtd[0,i] = oAtd[i]

    # compute potential
    potVar = compute_potential_variational(
        mLos,
        mRef,
        arrAtd,
        weight_pot,
    )

    # compute derivatives
    for i in [1,2,3]:
        dAtd.append(
            oAtd[i] @ skew_sym(sAnv[i,0] - oAnvErr[i])
        )
        dAnvErr.append(
            potVar[i] - (weight_kin[i,i]* skew_sym(sAnv[i,0])+ damp_arr[i,i]) @ oAnvErr[i]
        )

    # format output
    for i in [1, 2, 3]:
        dx.extend(dAtd[i].flatten())
    for i in [1, 2, 3]:
        dx.extend(dAnvErr[i].flatten())

    return dx


def statetime_to_dict(stateTime):

    # format data series
    atdSeries = dlib.DataSeries()
    anvSeries = dlib.DataSeries()

    # transform data
    for t in range(len(stateTime.T)):
        lAtd, lAnv = state_to_list(stateTime.T[t])

        # initialize Data Point
        tAtdData = dlib.AttitudeData(time=t, data=np.empty((4,4), object))
        tAnvData = dlib.AngularVelocityData(time=t, data=np.empty((4,4),object))

        # set data
        for i in [1,2,3]:
            tAtdData.data[(0,i)] = lAtd[i]
            tAnvData.data[(i,0)] = lAnv[i]

        # TODO make complete missing data from available
        tAtdData.complete_data()
        tAnvData.complete_data()

        # append to series
        atdSeries.series.append(copy.deepcopy(tAtdData))
        anvSeries.series.append(copy.deepcopy(tAnvData))

    outDict = {
        kpkg.ObsKeys.atd : atdSeries,
        kpkg.ObsKeys.anv : anvSeries
    }

    return outDict


# def constrained_var_estimator_continuous(**kwargs):
#     '''
#     Return the next observer internal state.

#     Based on the constrained attitude variational observer.

#     Returns
#     -------
#     dict
#         Observer internal state and debug parameters.

#     '''
#     estimates = {}

#     # get time step key and value
#     time_step : float = kwargs[kpkg.SettKeys.OPT_TIME_STEP]

#     # get observer values
#     measurement : dict = kwargs[kpkg.ObsKeys.KEY_MEAS]
#     old_estimate : dict = kwargs[kpkg.ObsKeys.KEY_OLD]
#     params : dict = kwargs[kpkg.ObsKeys.KEY_PAR]

#     # get prior observer state
#     old_est_atd = old_estimate[kpkg.ObsKeys.atd]
#     # old_est_anv = old_estimate[kpkg.ObsKeys.anv]
#     old_est_phi = old_estimate[kpkg.ObsKeys.anv_err]

#     # get observer parameters
#     obs_damp = params[kpkg.ObsKeys.obs_damp]
#     weight_pot = params[kpkg.ObsKeys.weight_pot]
#     weight_kin = params[kpkg.ObsKeys.weight_kin]

#     # get next measurement values
#     new_mea_anv : DataPoint = measurement[kpkg.SenKeys.anv]
#     new_mea_ref : DataPoint = measurement[kpkg.SenKeys.ref]
#     new_mea_los : DataPoint = measurement[kpkg.SenKeys.los]


#     # # get time vector
#     time = new_mea_anv.time - float(time_step)
#     tVec = [time, time + float(time_step)]

#     # tmAtd = 0
#     # tmAux = time.time()

#     # initialize ivp variable
#     ivpVar = []
#     # add attitutde to ivp variable
#     for idx in [(0,1), (0,2), (0,3)]:
#         ivpVar.extend(old_est_atd.data[idx].flatten())
#     # anv attitutde to ivp variable
#     for idx in [(1,0), (2,0), (3,0)]:
#         ivpVar.extend(old_est_phi.data[idx].flatten())

#     res = solve_ivp(
#         fun = filter_continuous,
#         t_span = [tVec[0],tVec[-1]],
#         y0 = ivpVar,
#         t_eval = tVec,
#         args = (
#             new_mea_los.data, new_mea_ref.data,
#             new_mea_anv.data, obs_damp.data,
#             weight_kin.data, weight_pot.data,
#         )
#     )

#     if res.success:
#         # get lists
#         lAtd, lAnv = state_to_list(res.y.T[-1])

#         # initialize Data Point
#         tAtdData = dlib.AttitudeData(time=float(res.t[-1]), data=np.empty((4,4), object))
#         tAnvData = dlib.AngularVelocityData(time=float(res.t[-1]), data=np.empty((4,4),object))

#         # set data
#         for i in [1,2,3]:
#             tAtdData.data[(0,i)] = lAtd[i]
#             tAnvData.data[(i,0)] = lAnv[i]

#         # TODO make complete missing data from available
#         tAtdData.complete_data()
#         tAnvData.complete_data()

#         estimates.update({
#             kpkg.ObsKeys.atd : tAtdData,
#             kpkg.ObsKeys.anv_err : tAnvData
#         })
#     else:
#         raise ValueError

#     # format observer internal angular velocities as ndarray
#     tmpAnv = np.empty((4,4), object)
#     tmpAnv[1,0] = new_mea_anv.data[1,0] - estimates[kpkg.ObsKeys.anv_err].data[1,0]
#     tmpAnv[2,0] = new_mea_anv.data[2,0] - estimates[kpkg.ObsKeys.anv_err].data[2,0]
#     tmpAnv[3,0] = new_mea_anv.data[3,0] - estimates[kpkg.ObsKeys.anv_err].data[3,0]

#     # format as an angular velocity point
#     new_obs_anv = AngularVelocityData(new_mea_anv.time)
#     new_obs_anv.setup_data(tmpAnv)

#     # save to output dictionary
#     estimates.update({
#         kpkg.ObsKeys.anv : new_obs_anv,
#     })

#     return estimates


def anv_discrete_abs(
        new_phi_list : np.ndarray,
        old_phi : np.ndarray,
        time_step : float,
        weight_kin : float,
        damp : np.ndarray,
        new_anv_sen : np.ndarray,
        new_pot : np.ndarray
    ) -> np.ndarray:

    # format new phi into an column vector array
    new_phi = np.array([
        [new_phi_list[0]],
        [new_phi_list[1]],
        [new_phi_list[2]]
    ])

    # compute residual
    fun_val =  (
        - weight_kin * new_phi
        + expm(-time_step * skew_sym( new_anv_sen - new_phi ))
        @ (
            time_step * new_pot
            + (weight_kin * np.eye(3) - time_step * damp) @ old_phi
        )
    )

    return np.linalg.norm(fun_val)**2


def get_principal_axis(size : int, i : int) -> np.ndarray:
    ''' Return principal axis i of orthonormal coordinate frame.

        Parameters
        ----------
        size : int
            Coordinate frame dimension.
        i : int
            Axis number.

        Returns
        -------
        np.ndarray
            Principal axis requested.
    '''
    # initialize axis
    axis = np.zeros((size,1), float)
    # set unit value
    axis[i,0] = 1
    return axis


def anv_discrete_abs_der(
        new_phi_list : np.ndarray,
        old_phi : np.ndarray,
        time_step : float,
        weight_kin : float,
        damp : np.ndarray,
        new_anv_sen : np.ndarray,
        new_pot : np.ndarray
    ) -> np.ndarray:

    # format new phi into an column vector array
    new_phi = np.array([
        [new_phi_list[0]],
        [new_phi_list[1]],
        [new_phi_list[2]]
    ])


    # compute residual
    fun_val =  (
        - weight_kin * new_phi
        + expm(-time_step * skew_sym( new_anv_sen - new_phi ))
        @ (
            time_step * new_pot
            + (weight_kin * np.eye(3) - time_step * damp) @ old_phi
        )
    )

    der = []
    for i in range(3):
        prn = get_principal_axis(3,i)

        fder =  (
            - weight_kin * prn
            + expm(-time_step * skew_sym( new_anv_sen - new_phi ))
            @ (-time_step * skew_sym( new_anv_sen - prn ) )
            @ (
                time_step * new_pot
                + (weight_kin * np.eye(3) - time_step * damp) @ old_phi
            )
        )
        der.append(2 * fun_val.T @ prn * fder)

    return der
