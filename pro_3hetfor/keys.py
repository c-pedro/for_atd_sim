''' Keys specific for subpackage problem and sensors. '''
from for_atd_sim.core.keys import SenKeys, ProKeys


class ProModelKeys(ProKeys):
    pass


class SenModelKeys(SenKeys):
    pass


pro_keys_list = [
    ProModelKeys.atd,
    ProModelKeys.anv,
    ProModelKeys.ilos,
    ProModelKeys.los,
    ProModelKeys.ref,
]

sen_keys_list = [
    SenModelKeys.anv,
    SenModelKeys.los,
    SenModelKeys.ref,
]

sen_cov_keys_list = [
    SenModelKeys.anv_cov,
    SenModelKeys.los_cov,
    SenModelKeys.ref_cov,
]

pro_sen_keymap = {
    ProModelKeys.anv : [SenModelKeys.anv, SenModelKeys.anv_cov],
    ProModelKeys.los : [SenModelKeys.los, SenModelKeys.los_cov],
    ProModelKeys.ref : [SenModelKeys.ref, SenModelKeys.ref_cov],
}
