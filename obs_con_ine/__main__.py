''' Constrained Variational Observer
'''
from pathlib import Path
from pro_3hetfor import keys
from for_atd_sim.core.simcell import SimCell
from for_atd_sim.core.anlcell import AnlCell

from for_atd_sim.obs_con_ine.model import setup_model
from for_atd_sim.pro_3hetfor.generate import random_degenerate_configuration
from for_atd_sim.pro_3hetfor.generate import random_degenerate_maneuver
from for_atd_sim.pro_3hetfor.generate import random_ambiguous_configuration
from for_atd_sim.obs_con_ine.keys import ObsModelKeys
import numpy as np

scell = SimCell(Path(__file__).parent)

scell.update_settings({})

# get degenerate configuration
deg_config = random_ambiguous_configuration()
deg_maneuver = { keys.ProModelKeys.manv : { # }}
        **random_degenerate_maneuver((1,2), [0,100], [0,300], [0,2*np.pi]),
        **random_degenerate_maneuver((2,0), [0,100], [0,300], [0,2*np.pi]),
    }
}

scell.update_input(deg_config | deg_maneuver)

setup_model(scell.model, scell.data.ini, scell.settings)
scell.id.set_new_id(50,1,0)
scell.fileman.clear_series_data(scell.settings.get_path_output(), 50)
scell.save_settings()
scell.save_parameters()

scell.get_true_values()
scell.get_measurements()
scell.get_estimates()

# scell.get_true_values(True)
# scell.get_measurements(True)
# scell.get_estimates(True)

acell = AnlCell(Path(__file__).parent)
acell.set_from_simcell(scell)

from for_atd_sim.lib_atd import trf_atd_error_angle_list
acell.transform_datalist([keys.ProKeys.atd, ObsModelKeys.atd], ObsModelKeys.atd_err_ang, trf_atd_error_angle_list)

from for_atd_sim.pro_3hetfor.metrics import degenerate_metric_trf, ambiguous_metric_trf
acell.transform_datadict([keys.ProKeys.ref, keys.ProKeys.ilos], keys.ProKeys.metric_deg, degenerate_metric_trf)

# test : plot simulation results
kw_list = [
    {'linestyle' : ist}
    for ist in ['--',':','-.', '--',':','-.', '--',':','-.']
]
acell.plot_var(keys.ProKeys.ilos, (1,2), 'test_ilos_true', kw_list)
acell.plot_var(keys.ProKeys.atd, (0,1), 'test_atd_true', kw_list)
# acell.plot_var(ObsModelKeys.atd, (0,1), 'test_atd_obs', kw_list)

acell.plot_multivar(
    [
        (ObsModelKeys.atd_err_ang, (0,1), {0:{}}),
        (ObsModelKeys.atd_err_ang, (0,2), {0:{}}),
        (ObsModelKeys.atd_err_ang, (0,3), {0:{}}),
        (keys.ProModelKeys.metric_deg, (1,2), {0:{}}),
        (keys.ProModelKeys.metric_deg, (1,12), {0:{}}),
        (keys.ProModelKeys.metric_deg, (2,12), {0:{}}),
        (keys.ProModelKeys.metric_deg, (1,3), {0:{}}),
        (keys.ProModelKeys.metric_deg, (1,13), {0:{}}),
        (keys.ProModelKeys.metric_deg, (3,13), {0:{}}),
    ],
    'test_atd_ang_err', 
    **{
        'yscale':'log',
        'legend': ['Q1', 'Q2', 'Q3', 'dg12', 'dg112', 'dg212', 'dg13', 'dg113', 'dg313' ],
    }
)



acell.plot_var(keys.ProKeys.anv, (1,0), 'test_anv', kw_list)
acell.plot_var(ObsModelKeys.anv, (1,0), 'test_anv_obs', kw_list)
acell.plot_var(ObsModelKeys.anv_err, (1,0), 'test_anv_err', kw_list)

print('Passed Ine')
