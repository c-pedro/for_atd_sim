''' Functions for problem setup. '''
import numpy as np

# keys
import for_atd_sim.pro_3hetfor.keys as kpro

# core classes
from for_atd_sim.core.simmodel import SimModel
from for_atd_sim.core.simsettings import SimSettings

# data structures
from for_atd_sim.lib_data.centralized_data import DataPoint
from for_atd_sim.lib_data.centralized_data import AttitudeData
from for_atd_sim.lib_data.centralized_data import AbsoluteDirectionData
from for_atd_sim.lib_data.centralized_data import RelativeDirectionData
from for_atd_sim.lib_data.centralized_data import AngularVelocityData
from for_atd_sim.lib_data.centralized_data import InertialRelativeDirectionData

# models
import for_atd_sim.lib_atd as latd

# other
import for_atd_sim.lib_math.random as rnd

# rename variable
# SKEYS = kpro.SettKeys


STD_MOI = {
    (0,1) : np.eye(3),
    (0,2) : np.eye(3),
    (0,3) : np.eye(3)
}

STD_TOR_GAIN = {
    (0,1) : 0.01,
    (0,2) : 0.01,
    (0,3) : 0.01
}

STD_TOR_FREQ = {
    (0,1) : 1,
    (0,2) : 1,
    (0,3) : 1
}

STD_TOR_PHASE = {
    (0,1) : 0,
    (0,2) : 0,
    (0,3) : 0
}

STD_TOR_IVAL = {
    (0,1) : rnd.random_unit_3d_vec(),
    (0,2) : rnd.random_unit_3d_vec(),
    (0,3) : rnd.random_unit_3d_vec()
}


def setup_problem(
        smodel : SimModel,
        input_data : dict,
        settings : SimSettings,
):
    ''' Setup problem from input. '''

    set_keys(smodel)

    # set initial true values
    set_init_true(smodel, input_data, settings)

    # set models and parameters
    setup_models(smodel, input_data)

    setup_manuever(smodel, input_data)

    smodel.problem.true_par.update({
        kpro.ProModelKeys.ivp_max_step : settings.ivp_precision
    })


def set_init_true(
        smodel : SimModel,
        input_data : dict,
        settings : SimSettings
    ):
    ''' Saves initial groundtruth in problem. '''

    # initialize data structures with initial time
    atd_data = AttitudeData()
    anv_data = AngularVelocityData()
    los_data = RelativeDirectionData()
    ref_data = AbsoluteDirectionData()
    ilos_data = InertialRelativeDirectionData()

    # set data in data structures
    atd_data.setup_data(input_data[kpro.ProKeys.atd])
    anv_data.setup_data(input_data[kpro.ProKeys.anv])
    los_data.setup_data(input_data[kpro.ProKeys.ilos], atd_data)
    ilos_data.setup_data(input_data[kpro.ProKeys.ilos])
    ref_data.setup_data(input_data[kpro.ProKeys.ref], atd_data)

    # set initial ground truth
    smodel.problem.true_init.update({kpro.ProKeys.atd : atd_data})
    smodel.problem.true_init.update({kpro.ProKeys.anv : anv_data})
    smodel.problem.true_init.update({kpro.ProKeys.los : los_data})
    smodel.problem.true_init.update({kpro.ProKeys.ref : ref_data})
    smodel.problem.true_init.update({kpro.ProKeys.ilos : ilos_data})


def setup_models(smodel : SimModel, input_data : dict):
    ''' Saves models and respective parameters in problem. '''
    par_moi = DataPoint()
    tor_ival_dp = DataPoint()
    tor_gain_dp = DataPoint()
    tor_freq_dp = DataPoint()
    tor_phase_dp = DataPoint()

    # parse kinematic parameters
    par_moi.data = input_data.get(kpro.ProKeys.moi, STD_MOI)

    # torque function
    tor_fun_str = input_data.get(latd.KEY_TOR_FUN, {None : latd.KEY_TORFUN_STA})

    # get function from dict
    tor_fun = latd.tor_fun_db[tor_fun_str[None]]
    tor_ival = input_data.get(latd.KEY_TOR_PAR_INI, STD_TOR_IVAL)
    tor_gain = input_data.get(latd.KEY_TOR_PAR_GAIN, STD_TOR_GAIN)
    tor_freq = input_data.get(latd.KEY_TOR_PAR_FREQ, STD_TOR_FREQ)
    tor_phase = input_data.get(latd.KEY_TOR_PAR_PHASE, STD_TOR_PHASE)

    tor_ival_dp.data = tor_ival
    tor_gain_dp.data = tor_gain
    tor_freq_dp.data = tor_freq
    tor_phase_dp.data = tor_phase

    # set formation motion
    smodel.problem.true_par.update({
        kpro.ProKeys.moi : par_moi,
        latd.KEY_TOR_FUN : tor_fun,
        latd.KEY_TOR_PAR_INI : tor_ival_dp,
        latd.KEY_TOR_PAR_GAIN : tor_gain_dp,
        latd.KEY_TOR_PAR_FREQ : tor_freq_dp,
        latd.KEY_TOR_PAR_PHASE : tor_phase_dp,
    })

    # set global problem function
    smodel.problem.true_model = latd.model_dynamics_het


def setup_manuever(smodel : SimModel, input_data : dict):
    ''' Saves manuevers from input in problem as DataPoints. '''
    manv = DataPoint()

    manv.data =  {k: np.array(v, object) for (k,v) in input_data[kpro.ProKeys.manv].items()}

    smodel.problem.true_par.update([(kpro.ProKeys.manv, manv)])


def set_keys(smodel : SimModel):
    smodel.pro_keys = kpro.pro_keys_list
