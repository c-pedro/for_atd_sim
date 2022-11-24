''' Observer setup and simulation functions. '''
import copy
import numpy as np

# third party
from scipy.integrate import solve_ivp

# core classes
import for_atd_sim.core.observer as obs
import for_atd_sim.core.problem as pro
from for_atd_sim.core.problem import Problem
from for_atd_sim.core.observer import Observer
from for_atd_sim.core.sensors import Sensors

# libraries - package
import for_atd_sim.lib_math as lmat
import for_atd_sim.lib_atd.torque as torque
import for_atd_sim.lib_atd.motion as motion

# data structure - package
import for_atd_sim.lib_data as dlib
from for_atd_sim.lib_data.basic_data import DataPoint
from for_atd_sim.lib_data.basic_data import DataSeries
from for_atd_sim.lib_data.centralized_data import AttitudeData
from for_atd_sim.lib_data.centralized_data import AngularVelocityData

from for_atd_sim.core.simmodel import SimModel
from for_atd_sim.core.simsettings import SimSettings

# subpackage
import for_atd_sim.obs_con_ine.keys as kpkg

from for_atd_sim.obs_con_ine import model
import for_atd_sim.obs_con_ine.estimator as estim


def setup_observer(smodel : SimModel, input_data : dict, settings : SimSettings) -> None:
    ''' Setup observer from input.

        Parameters
        ----------
        smodel : SimModel
            Simulation model instance.
        input_data : dict
            Observer initial state and parameters.
    '''
    time_0 = settings.time_init
    
    # set keys for observer variables
    smodel.obs_keys = kpkg.obs_keys
    smodel.obs_cov_keys = kpkg.obs_cov_keys
    
    # set estimator module
    smodel.observer.obs_model = estim.constrained_var_estimator

    # parse groundtruth
    anv_data : AngularVelocityData = smodel.problem.true_init[kpkg.kpro.ProModelKeys.anv]

    # create parameter data points
    wkin_pnt = DataPoint(time_0)
    wpot_pnt = DataPoint(time_0)
    damp_pnt = DataPoint(time_0)

    wkin_pnt.data = input_data[kpkg.ObsModelKeys.weight_kin]
    wpot_pnt.data = input_data[kpkg.ObsModelKeys.weight_pot]
    damp_pnt.data = input_data[kpkg.ObsModelKeys.obs_damp]

    # initial values
    obs_atd = AttitudeData(time_0)
    obs_anv = AngularVelocityData(time_0)
    obs_phi = AngularVelocityData(time_0)

    obs_atd.setup_data(input_data[kpkg.ObsModelKeys.atd])

    # setup observer angular velocity and error
    if kpkg.ObsModelKeys.anv in input_data:
        obs_anv.setup_data(input_data[kpkg.ObsModelKeys.anv])

        for index in obs_anv.data:
            obs_phi.data[index] = anv_data.data[index] - obs_anv.data[index]

    elif kpkg.ObsModelKeys.anv_err in input_data:
        obs_phi.setup_data(input_data[kpkg.ObsModelKeys.anv_err])

        for index in obs_phi.data:
            obs_anv.data[index] = anv_data.data[index] - obs_phi.data[index]

    else:
        raise ValueError

    # set observer parameters
    smodel.observer.obs_par.update({ kpkg.ObsModelKeys.obs_damp :   damp_pnt})
    smodel.observer.obs_par.update({ kpkg.ObsModelKeys.weight_pot : wpot_pnt})
    smodel.observer.obs_par.update({ kpkg.ObsModelKeys.weight_kin : wkin_pnt})
    smodel.observer.obs_par.update({ kpkg.ObsModelKeys.time_step : settings.time_step})

    # set observer initial estimations
    smodel.observer.obs_state.update({ kpkg.ObsModelKeys.atd : obs_atd})
    smodel.observer.obs_state.update({ kpkg.ObsModelKeys.anv : obs_anv})
    smodel.observer.obs_state.update({ kpkg.ObsModelKeys.anv_err : obs_phi})

    # set observer initial estimations
    smodel.observer.obs_init.update({ kpkg.ObsModelKeys.atd : obs_atd})
    smodel.observer.obs_init.update({ kpkg.ObsModelKeys.anv : obs_anv})
    smodel.observer.obs_init.update({ kpkg.ObsModelKeys.anv_err : obs_phi})




def set_observer_error(data_input : dict, in_rot_err : list, in_anv_err : list):
    ''' Return data updated with a given observer error. '''

    # initialize observer error matrices
    atd_err = np.empty((4,4), object)
    atd_err[0,1] = lmat.rot_mtx(
        np.deg2rad(in_rot_err[0]), lmat.random_unit_3d_vec()
    )
    atd_err[0,2] = lmat.rot_mtx(
        np.deg2rad(in_rot_err[1]), lmat.random_unit_3d_vec()
    )
    atd_err[0,3] = lmat.rot_mtx(
        np.deg2rad(in_rot_err[2]), lmat.random_unit_3d_vec()
    )

    # initialize attitude estimate
    obs_atd = np.empty((4,4), object)
    obs_atd[0,1] = atd_err[0,1].T @ data_input[kpkg.ProKeys.atd][0,1]
    obs_atd[0,2] = atd_err[0,2].T @ data_input[kpkg.ProKeys.atd][0,2]
    obs_atd[0,3] = atd_err[0,3].T @ data_input[kpkg.ProKeys.atd][0,3]

    # initialize attitude estimate
    anv_err = np.empty((4,1), object)
    anv_err[1,0] = lmat.random_unit_3d_vec() * np.deg2rad(in_anv_err[0])
    anv_err[2,0] = lmat.random_unit_3d_vec() * np.deg2rad(in_anv_err[1])
    anv_err[3,0] = lmat.random_unit_3d_vec() * np.deg2rad(in_anv_err[2])

    data_input[kpkg.ObsKeys.atd] = obs_atd
    data_input[kpkg.ObsKeys.anv_err] = anv_err

    return data_input


def sample_observer_error(
        data_input : dict,
        in_rot_mag : list,
        in_anv_mag : list
    ):
    ''' Return input data with updated observer errors.'''
    # sample errors
    in_rot_err = 2 * np.random.default_rng().random((3)) - 1
    in_anv_err = 2 * np.random.default_rng().random((3)) - 1

    in_rot_err = in_rot_mag * in_rot_err / np.linalg.norm(in_rot_err)
    in_anv_err = in_anv_mag * in_anv_err / np.linalg.norm(in_anv_err)

    # initialize observer error matrices
    atd_err = np.empty((4,4), object)
    atd_err[0,1] = lmat.rot_mtx(
        np.deg2rad(in_rot_err[0]), lmat.random_unit_3d_vec()
    )
    atd_err[0,2] = lmat.rot_mtx(
        np.deg2rad(in_rot_err[1]), lmat.random_unit_3d_vec()
    )
    atd_err[0,3] = lmat.rot_mtx(
        np.deg2rad(in_rot_err[2]), lmat.random_unit_3d_vec()
    )

    # initialize attitude estimate
    obs_atd = np.empty((4,4), object)
    obs_atd[0,1] = atd_err[0,1].T @ data_input[kpkg.ProKeys.atd][0,1]
    obs_atd[0,2] = atd_err[0,2].T @ data_input[kpkg.ProKeys.atd][0,2]
    obs_atd[0,3] = atd_err[0,3].T @ data_input[kpkg.ProKeys.atd][0,3]

    # initialize attitude estimate
    anv_err = np.empty((4,1), object)
    anv_err[1,0] = lmat.random_unit_3d_vec() * np.deg2rad(in_anv_err[0])
    anv_err[2,0] = lmat.random_unit_3d_vec() * np.deg2rad(in_anv_err[1])
    anv_err[3,0] = lmat.random_unit_3d_vec() * np.deg2rad(in_anv_err[2])

    anv = np.empty((4,1), object)
    anv[1,0] = data_input[kpkg.ProKeys.anv][1,0] - anv_err[1,0]
    anv[2,0] = data_input[kpkg.ProKeys.anv][2,0] - anv_err[2,0]
    anv[3,0] = data_input[kpkg.ProKeys.anv][3,0] - anv_err[3,0]

    data_input[kpkg.ObsKeys.atd] = obs_atd
    data_input[kpkg.ObsKeys.anv_err] = anv_err
    data_input[kpkg.ObsKeys.anv] = anv

    return data_input


def sample_observer_error_norm(
        data_input : dict,
        in_rot_max : float,
        in_anv_max : float
    ):
    ''' Return input data with updated observer errors.'''

    # sample max error indexes
    max_atd_idx = np.random.default_rng().choice(3)
    max_anv_idx = np.random.default_rng().choice(3)

    # sample attitude principal angle errors
    in_rot_err = np.deg2rad(in_rot_max) * np.random.default_rng().random((3))
    in_rot_err[max_atd_idx] = np.deg2rad(in_rot_max)

    # sample angular velocity error magnitudes
    in_anv_err = np.deg2rad(in_anv_max) * np.random.default_rng().random((3))
    in_anv_err[max_anv_idx] = np.deg2rad(in_anv_max)

    # initialize observer arrays
    atd_err = np.empty((4,4), object)
    obs_atd = np.empty((4,4), object)
    anv_err = np.empty((4,1), object)
    anv = np.empty((4,1), object)

    for i in [1,2,3]:
        # initialize observer error matrices
        atd_err[0,i] = lmat.rot_mtx(in_rot_err[i-1], lmat.random_unit_3d_vec())

        # initialize attitude estimate
        obs_atd[0,i] = atd_err[0,i].T @ data_input[kpkg.ProKeys.atd][0,i]

        # initialize angular velocity estimate errror
        anv_err[i,0] = in_anv_err[i-1] * lmat.random_unit_3d_vec()

        anv[i,0] = data_input[kpkg.ProKeys.anv][i,0] - anv_err[i,0]

    # save in input data object
    data_input[kpkg.ObsKeys.atd] = obs_atd
    data_input[kpkg.ObsKeys.anv_err] = anv_err
    data_input[kpkg.ObsKeys.anv] = anv

    return data_input


def parse_obs(**kwargs):
    ''' TODO'''
    return kwargs


def statetime_to_dict(state_time, time_vec):
    ''' Return state as a dict. '''

    # initialize data series
    atd_pro = dlib.DataSeries()
    atd_obs = dlib.DataSeries()
    anv_pro = dlib.DataSeries()
    anverr_obs = dlib.DataSeries()

    # transform data
    for i, _ in enumerate(state_time.T):
        state_tuple = model.state_to_list(state_time.T[i])
        atd_pro_list = state_tuple[0]
        atd_obs_list = state_tuple[1]
        anv_pro_list = state_tuple[2]
        anverr_obs_list = state_tuple[3]

        # initialize Data Point
        atd_pro_pnt = dlib.AttitudeData(
            time=time_vec[i], data=np.empty((4,4), object)
        )
        atd_obs_pnt = dlib.AttitudeData(
            time=time_vec[i], data=np.empty((4,4), object)
        )
        anv_pro_pnt = dlib.AngularVelocityData(
            time=time_vec[i], data=np.empty((4,4), object)
        )
        anverr_obs_pnt = dlib.AngularVelocityData(
            time=time_vec[i], data=np.empty((4,4), object)
        )

        # set data
        for j in [1,2,3]:
            atd_pro_pnt.data[(0,j)] = atd_pro_list[j]
            atd_obs_pnt.data[(0,j)] = atd_obs_list[j]
            anv_pro_pnt.data[(j,0)] = anv_pro_list[j]
            anverr_obs_pnt.data[(j,0)] = anverr_obs_list[j]

        # TODO : make complete missing data from available
        atd_pro_pnt.complete_data()
        atd_obs_pnt.complete_data()
        anv_pro_pnt.complete_data()
        anverr_obs_pnt.complete_data()

        # append to series
        atd_pro.series.append(copy.deepcopy(atd_pro_pnt))
        atd_obs.series.append(copy.deepcopy(atd_obs_pnt))
        anv_pro.series.append(copy.deepcopy(anv_pro_pnt))
        anverr_obs.series.append(copy.deepcopy(anverr_obs_pnt))

    out_dict = {
        pro.ProKeys.atd : atd_pro,
        pro.ProKeys.anv : anv_pro,
        obs.ObsKeys.atd : atd_obs,
        obs.ObsKeys.anv_err : anverr_obs
    }

    return out_dict


# def complete_simulation(
#         tvec : list,
#         sim_pro : Problem,
#         sim_sen : Sensors,
#         sim_obs : Observer,
#         **kwargs
#     ) -> dict:
#     ''' Execute continuous simulation and return groundtruth and observer vars.
#     '''
#     # parse max step
#     if kpkg.SettKeys.OPT_IVP_PRECISION in kwargs:
#         my_max_step = kwargs[kpkg.SettKeys.OPT_IVP_PRECISION]
#     else:
#         my_max_step = 0.1

#     # parse moment of inertia
#     moi_val = moi.parse_moi(**sim_pro.true_par)

#     # parse torque
#     tor_fun, tor_par = torque.parse_tor(**sim_pro.true_par)

#     # parse potential
#     pot_fun, pot_par = pot.parse_pot(**sim_pro.true_par)

#     # parse observer params
#     obs_par = sim_obs.obs_params

#     # initialize ivp variable
#     ivp_var = []
#     # add attitutde to ivp variable
#     for idx in [(0,1), (0,2), (0,3)]:
#         ivp_var.extend(sim_pro.true_init[pro.ProKeys.atd].data[idx].flatten())
#     for idx in [(0,1), (0,2), (0,3)]:
#         ivp_var.extend(sim_obs.obs_state[obs.ObsKeys.atd].data[idx].flatten())

#     # anv attitutde to ivp variable
#     for idx in [(1,0), (2,0), (3,0)]:
#         ivp_var.extend(sim_pro.true_init[pro.ProKeys.anv].data[idx].flatten())
#     for idx in [(1,0), (2,0), (3,0)]:
#         ivp_var.extend(sim_obs.obs_state[obs.ObsKeys.anv_err].data[idx].flatten())

#     # compute attitude and angular velocity
#     ivp_res = solve_ivp(
#         fun = model.complete_model,
#         t_span = [tvec[0], tvec[-1]],
#         y0 = ivp_var,
#         t_eval = tvec,
#         args = (
#             moi_val,
#             tor_fun, tor_par,
#             pot_fun, pot_par,
#             obs_par, sim_pro.true_init,
#             sim_sen
#         ),
#         max_step=my_max_step,
#         # rtol=10**-8,
#         # atol=10**-8
#     )

#     # initialize problem measurements
#     ref_ser = DataSeries()
#     los_ser = DataSeries()
#     ilos_ser = DataSeries()
#     pot_ser = DataSeries()

#     # in case of sucess return results
#     if ivp_res.success:
#         # get state dict
#         out_dict = statetime_to_dict(ivp_res.y, tvec)

#         # compute problem variables
#         for j, time in enumerate(tvec):

#             # parse maneuver manager
#             manv_mng = motion.parse_manv(**pot_par)

#             # format attitude as DataPoint
#             aux_mtx = np.empty((4,4), object)
#             for idx in [(0,1), (0,2), (0,3)]:
#                 aux_mtx[idx] = out_dict[kpkg.ProKeys.atd].series[j].data[idx]
#             atd_pnt = dlib.AttitudeData(time, aux_mtx)
#             atd_pnt.complete_data()

#             # parse vectors
#             mea_vectors = motion.motion_formation(
#                 time, sim_pro.true_init,
#                 atd_pnt, manv_mng
#             )

#             # parse
#             atd_pro = [None]
#             atd_obs = [None]
#             for idx in [(0,1), (0,2), (0,3)]:
#                 atd_pro.append(atd_pnt.data[idx])
#                 atd_obs.append(out_dict[kpkg.ObsKeys.atd].series[j].data[idx])

#             # compute potential
#             pot_val = np.empty(4, object)
#             pot_val = pot_fun(time, atd_pro, atd_obs, sim_pro.true_init, sim_sen, **pot_par)

#             # append to series
#             ref_ser.append_data(copy.deepcopy(mea_vectors[kpkg.ProKeys.ref]))
#             los_ser.append_data(copy.deepcopy(mea_vectors[kpkg.ProKeys.los]))
#             ilos_ser.append_data(copy.deepcopy(mea_vectors[kpkg.ProKeys.ilos]))
#             pot_ser.series.append(copy.deepcopy(DataPoint(time, pot_val)))

#         # append true measuements
#         out_dict[kpkg.ProKeys.ref] = ref_ser
#         out_dict[kpkg.ProKeys.los] = los_ser
#         out_dict[kpkg.ProKeys.ilos] = ilos_ser
#         out_dict[kpkg.ProKeys.pot] = pot_ser

#         return out_dict

#     raise ValueError
