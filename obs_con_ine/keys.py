from for_atd_sim.core.keys import ObsKeys
import for_atd_sim.pro_3hetfor.keys as kpro


class ObsModelKeys(ObsKeys):
    obs_damp = ObsKeys.prefix + ObsKeys.get_key('damping', ObsKeys.KEY_VAL)
    weight_kin = ObsKeys.prefix + ObsKeys.get_key('wkinetic', ObsKeys.KEY_VAL)
    weight_pot = ObsKeys.prefix + ObsKeys.get_key('wpotential', ObsKeys.KEY_VAL)
    time_step = ObsKeys.prefix +ObsKeys.get_key('time','step')

    percent_converged_trials = ObsKeys.prefix +ObsKeys.get_key('percent', 'converged','trials')
    not_converged_trials = ObsKeys.prefix +ObsKeys.get_key('not', 'converged','trials')


obs_keys = [
    ObsModelKeys.atd,
    ObsModelKeys.anv,
    ObsModelKeys.anv_err,
]

obs_cov_keys = [
    ObsModelKeys.atd_cov
]