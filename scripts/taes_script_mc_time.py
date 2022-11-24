''' Convergence Analysis with Time

'''
import for_atd_sim.scripts.script_path
import numpy as np
from pathlib import Path

from for_atd_sim.obs_con_ine.keys import ObsModelKeys

from for_atd_sim.core.simid import SimID
from for_atd_sim.core.simsettings import SimSettings
from for_atd_sim.lib_io import FileManager
from for_atd_sim.obs_con_ine.model import setup_model
from for_atd_sim.obs_con_ine.mc_generate import taes_random_fixed_error
from for_atd_sim.obs_con_ine.mc_anl import basic_trf_anl
from lib_post.stats import compute_mean
from lib_mcarlo import MonteCarlo
from lib_mcarlo import get_not_converged

import for_atd_sim.lib_gui.terminal as gui

gui.init_msg('Convergence Analysis with Time')
con_pkg_path = Path(Path(__file__).parent.parent, 'obs_con_ine')
sim_mcarlo = MonteCarlo(con_pkg_path)

gui.write_section('Setting Parameters')
# set mc parameters
mc_ser = 200
mc_mea = 0
n_trials = 40
sim_mcarlo.use_ncores = 3

# set simulation parameters
mc_settings = SimSettings(con_pkg_path)
mc_settings.time_init = 0
mc_settings.time_final = 200
mc_settings.time_step = 0.2
mc_settings.ivp_precision = 0.03

# set Monte Carlo functions
sim_mcarlo.function_generate = taes_random_fixed_error
sim_mcarlo.function_anl = basic_trf_anl
sim_mcarlo.function_setup = setup_model

# set Monte Carlo arguments
sim_mcarlo.args_generate = [np.rad2deg(2*np.pi/3), np.rad2deg(np.pi/18)]
sim_mcarlo.args_settings = mc_settings

# define ids
id_list = [SimID(mc_ser, tru_id, mc_mea) for tru_id in range(n_trials)]

# execute Monte Carlo trials
gui.write_section('Running Simulations')
sim_mcarlo.run_mc(id_list)

# compute mean error
gui.write_section('Computing Mean')
compute_mean(mc_ser, mc_mea, ObsModelKeys.atd_err_ang_norm, ObsModelKeys.mean_atd_err_ang_norm, con_pkg_path)
compute_mean(mc_ser, mc_mea, ObsModelKeys.anv_err_norm, ObsModelKeys.mean_anv_err_norm, con_pkg_path)


# check convergence at specified instants
gui.write_section('Checking Convergence')
mc_conv_lim = np.pi/360
# times = [t for t in range(100, 1000, 100)]
times = [t for t in range(10, 200, 20)] # DEBUG
conv_percent = {}

for t in times:
    not_conv_list = get_not_converged(ObsModelKeys.atd_err_ang_norm, (0,0), t, id_list, mc_conv_lim, con_pkg_path)
    conv_percent[t] = 100 * (len(id_list) - len(not_conv_list)) / len(id_list)

# save percentage of conveged trials
aux_fman = FileManager()
aux_fman.save_csv_dict(mc_settings.get_path_output(), SimID(mc_ser), conv_percent, ObsModelKeys.percent_converged_trials, aux_fman.KEY_TAG_ANL)


gui.end_msg('Finished Convergence Analysis')
