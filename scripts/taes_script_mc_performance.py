''' Convergence Analysis with Time

'''
import for_atd_sim.scripts.script_path
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from for_atd_sim.core.simid import SimID
from for_atd_sim.core.simcell import SimCell
from for_atd_sim.core.anlcell import AnlCell
from for_atd_sim.core.simsettings import SimSettings
from for_atd_sim.lib_io import FileManager

from lib_mcarlo import MonteCarlo
from lib_mcarlo import get_not_converged
from lib_post.stats import compute_converged_mean
import for_atd_sim.lib_gui.terminal as gui

from for_atd_sim.obs_con_ine.keys import ObsModelKeys
from for_atd_sim.obs_con_ine.model import setup_model 
from for_atd_sim.obs_con_ine.mc_generate import taes_random_fixed_error
from for_atd_sim.obs_con_ine.mc_anl import basic_trf_anl, basic_det_trf_anl

from for_atd_sim.obs_atd_ine.model import setup_model as atdobs_setup
from for_atd_sim.obs_detatd_inerel.model import setup_model as detobs_setup


gui.init_msg('Performance Analysis')
con_pkg_path = Path(Path(__file__).parent.parent, 'obs_con_ine')
atd_spkg_path = Path(Path(__file__).parent.parent, 'obs_atd_ine')
det_pkg_path = Path(Path(__file__).parent.parent, 'obs_detatd_inerel')
sim_mcarlo = MonteCarlo(con_pkg_path)

gui.write_section('Setting Parameters')
# set mc parameters
mc_ser = 100
mc_mea = 0
n_trials = 50

# set simulation parameters
mc_settings = SimSettings(con_pkg_path)
mc_settings.time_init = 0
mc_settings.time_final = 100
mc_settings.time_step = 0.2
mc_settings.ivp_precision = 0.03

# set Monte Carlo functions
sim_mcarlo.use_ncores = 5
sim_mcarlo.function_generate = taes_random_fixed_error
sim_mcarlo.function_anl = basic_trf_anl
sim_mcarlo.function_setup = setup_model

# set Monte Carlo arguments
sim_mcarlo.args_generate = [np.rad2deg(2*np.pi/3), np.rad2deg(np.pi/18)]
sim_mcarlo.args_settings = mc_settings

# define ids
id_list = [SimID(mc_ser, tru_id, mc_mea) for tru_id in range(n_trials)]

# clean existing results
FileManager().clear_series_data(mc_settings.get_path_output(), mc_ser)

# execute Monte Carlo trials
gui.write_section('Running Constrained Observer Simulations')
sim_mcarlo.run_mc(id_list)

# check convergence at specified instants
gui.write_section('Checking Convergence')
mc_conv_lim = np.pi / 3600
times = [mc_settings.time_final]
conv_percent = {}

for t in times:
    not_conv_list = get_not_converged(ObsModelKeys.atd_err_ang_norm, (0,0), t, id_list, mc_conv_lim, con_pkg_path)
    conv_percent[t] = 100 * (len(id_list) - len(not_conv_list)) / len(id_list)

# save percentage of conveged trials
aux_fman = FileManager()
aux_fman.save_csv_dict(mc_settings.get_path_output(), SimID(mc_ser), conv_percent, ObsModelKeys.percent_converged_trials, aux_fman.KEY_TAG_ANL)
aux_fman.save_csv_dict(mc_settings.get_path_output(), SimID(mc_ser), {ObsModelKeys.not_converged_trials : not_conv_list}, ObsModelKeys.not_converged_trials, aux_fman.KEY_TAG_ANL)

# compute mean error
gui.write_section('Computing Mean')
compute_converged_mean(mc_ser, mc_mea, ObsModelKeys.atd_err_ang_norm, ObsModelKeys.mean_conv_atd_err_ang_norm, con_pkg_path, not_conv_list)
compute_converged_mean(mc_ser, mc_mea, ObsModelKeys.anv_err_norm, ObsModelKeys.mean_conv_anv_err_norm, con_pkg_path, not_conv_list)


# Attitude Reconstruction Observer
gui.write_section('Attitude Reconstruction')
# path 



# import data
atd_cell = SimCell(atd_spkg_path)

atd_cell.id = SimID(mc_ser)
atd_cell.import_series_data(mc_settings.get_path_output())

atd_mc = MonteCarlo(atd_spkg_path)

# set simulation parameters
atd_mc_settings = SimSettings(atd_spkg_path)
atd_mc_settings.time_init = mc_settings.time_init
atd_mc_settings.time_final = mc_settings.time_final
atd_mc_settings.time_step = mc_settings.time_step
atd_mc_settings.ivp_precision = mc_settings.ivp_precision

# set Monte Carlo functions
atd_mc.use_ncores = 5
atd_mc.function_anl = basic_trf_anl
atd_mc.function_setup = atdobs_setup

atd_mc.load_sett = True
atd_mc.load_pars = True
atd_mc.load_true = True
atd_mc.load_meas = True
atd_mc.run_mc(id_list)

# check convergence at specified instants
gui.write_section('Checking Convergence')
mc_conv_lim = np.pi / 3600
times = [atd_mc_settings.time_final]
conv_percent = {}

for t in times:
    atd_not_conv_list = get_not_converged(ObsModelKeys.atd_err_ang_norm, (0,0), t, id_list, mc_conv_lim, atd_spkg_path)
    conv_percent[t] = 100 * (len(id_list) - len(atd_not_conv_list)) / len(id_list)

# save percentage of conveged trials
aux_fman = FileManager()
aux_fman.save_csv_dict(atd_mc_settings.get_path_output(), SimID(mc_ser), conv_percent, ObsModelKeys.percent_converged_trials, aux_fman.KEY_TAG_ANL)
aux_fman.save_csv_dict(atd_mc_settings.get_path_output(), SimID(mc_ser), {ObsModelKeys.not_converged_trials : not_conv_list}, ObsModelKeys.not_converged_trials, aux_fman.KEY_TAG_ANL)

# compute mean error
gui.write_section('Computing Mean')
compute_converged_mean(mc_ser, mc_mea, ObsModelKeys.atd_err_ang_norm, ObsModelKeys.mean_conv_atd_err_ang_norm, atd_spkg_path, not_conv_list)
compute_converged_mean(mc_ser, mc_mea, ObsModelKeys.anv_err_norm, ObsModelKeys.mean_conv_anv_err_norm, atd_spkg_path, not_conv_list)


# Deterministic Attitude 
gui.write_section('Deterministic Attitude')
# import data
det_cell = SimCell(det_pkg_path)
det_cell.id = SimID(mc_ser)
det_cell.import_series_data(mc_settings.get_path_output())

# set monte carlo
det_mc = MonteCarlo(det_pkg_path)

# set simulation parameters
det_mc_settings = SimSettings(det_pkg_path)
det_mc_settings.time_init = mc_settings.time_init
det_mc_settings.time_final = mc_settings.time_final
det_mc_settings.time_step = mc_settings.time_step
det_mc_settings.ivp_precision = mc_settings.ivp_precision

# set Monte Carlo functions
det_mc.use_ncores = 5
det_mc.function_anl = basic_det_trf_anl
det_mc.function_setup = detobs_setup

det_mc.load_sett = True
det_mc.load_pars = True
det_mc.load_true = True
det_mc.load_meas = True
det_mc.run_mc(id_list)

# compute mean error
gui.write_section('Computing Mean')
compute_converged_mean(mc_ser, mc_mea, ObsModelKeys.atd_err_ang_norm, ObsModelKeys.mean_conv_atd_err_ang_norm, det_pkg_path, not_conv_list)


# plot results
gui.write_bounded('Plotting Results')
atd_acell = AnlCell(atd_spkg_path)
con_acell = AnlCell(con_pkg_path)
det_acell = AnlCell(det_pkg_path)

plt.figure(1)
# load mean error
atd_acell.id = SimID(mc_ser)
con_acell.id = SimID(mc_ser)
det_acell.id = SimID(mc_ser)

atd_acell.data.anl.update(**atd_acell.load_results([ObsModelKeys.mean_conv_atd_err_ang_norm]))
con_acell.data.anl.update(**con_acell.load_results([ObsModelKeys.mean_conv_atd_err_ang_norm]))
det_acell.data.anl.update(**det_acell.load_results([ObsModelKeys.mean_conv_atd_err_ang_norm]))

# save
atd_tarr, atd_darr = atd_acell.data.anl[ObsModelKeys.mean_conv_atd_err_ang_norm].get_plot_arrays((0,0))
con_tarr, con_darr = con_acell.data.anl[ObsModelKeys.mean_conv_atd_err_ang_norm].get_plot_arrays((0,0))
det_tarr, det_darr = det_acell.data.anl[ObsModelKeys.mean_conv_atd_err_ang_norm].get_plot_arrays((0,0))

atd_obj = {'time' : atd_tarr, 'reconstructed' : atd_darr[0]}
con_obj = {'time' : con_tarr, 'constraint' : con_darr[0]}
det_obj = {'time' : det_tarr, 'deterministic' : det_darr[0]}

plt.plot('time', 'reconstructed', data=atd_obj)
plt.plot('time', 'constraint', data=con_obj)
plt.plot('time', 'deterministic', data=det_obj)
    
    
plt.xlabel('Time')
plt.ylabel('Attitude Error')
plt.legend()
plt.yscale('log')
plt.savefig(Path(mc_settings.get_path_output(), 'performance-anl.png'))


gui.end_msg('Finished Performance Analysis')
