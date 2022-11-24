from for_atd_sim.core.keys import ObsKeys
from for_atd_sim.pro_3hetfor.keys import SenModelKeys


KEY_METRICS_DEGEN = 'anl-metrics-degen'
KEY_METRICS_AMB = 'anl-metrics-amb'

OPT_PROCRUSTES = 'option-procrustes'
OPT_COVARIANCE = 'option-covariance'


class ObsModelKeys(ObsKeys):
    pass


obs_keys = [
    ObsModelKeys.atd,
]

obs_cov_keys = [
    ObsModelKeys.atd_cov
]
