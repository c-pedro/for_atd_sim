''' Initialize, setup, and manage simulation for variational observer 3.
'''
# standard libraries
import numpy as np

# simulation - data
# from for_atd_sim.lib_data import import_data
from for_atd_sim.lib_data.centralized_data import DataPoint
from for_atd_sim.lib_data.centralized_data import AttitudeData
from for_atd_sim.lib_data.centralized_data import AngularVelocityData

from for_atd_sim.core.problem import Problem
from for_atd_sim.core.observer import Observer

# simulation - estimation
from for_atd_sim.obs_atd_ine.estimator import variational_estimator_dsc_ind

# package keys
import for_atd_sim.obs_atd_ine.keys as kpkg


# import for_atd_sim.obs_atd_ine.potential as pot
from for_atd_sim.obs_atd_ine import model
import for_atd_sim.obs_atd_ine.estimator as estim

from for_atd_sim.core.simmodel import SimModel
from for_atd_sim.core.simsettings import SimSettings


OBS_SETTINGS = 'obssettings'

def setup_observer(smodel : SimModel, input_data : dict, settings : SimSettings) -> None:
    ''' Setup observer from input.

        Parameters
        ----------
        inputData : dict
            Observer initial state and parameters.

        Side Effects
        ------------
        1. Change simulation Observer instance data.

        Restrictions
        ------------
        1. Data must exist and be compatible.

    '''
    

    time_0 = settings.time_init
    
    # set keys for observer variables
    smodel.obs_keys = kpkg.obs_keys
    smodel.obs_cov_keys = kpkg.obs_cov_keys

    # set estimator module
    smodel.observer.obs_model = estim.variational_estimator_dsc_ind 
    # smodel.observer.obs_model = estim.variational_estimator_weights # weights option

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
    

