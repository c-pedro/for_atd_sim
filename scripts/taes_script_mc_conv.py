''' Convergence Analysis with Error

'''
import for_atd_sim.scripts.script_path
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from for_atd_sim.core.anlcell import AnlCell
from for_atd_sim.core.simid import SimID
from for_atd_sim.core.simsettings import SimSettings

from for_atd_sim.lib_io import FileManager
from lib_post.stats import compute_converged_mean
from lib_mcarlo import MonteCarlo
from lib_mcarlo import get_not_converged
import for_atd_sim.lib_gui.terminal as gui

from for_atd_sim.obs_con_ine.keys import ObsModelKeys
from for_atd_sim.obs_con_ine.model import setup_model
from for_atd_sim.obs_con_ine.mc_generate import taes_random_fixed_error
from for_atd_sim.obs_con_ine.mc_anl import basic_trf_anl


gui.init_msg('Convergence Analysis with Error')
con_pkg_path = Path(Path(__file__).parent.parent, 'obs_con_ine')
sim_mcarlo = MonteCarlo(con_pkg_path)

gui.write_section('Setting Parameters')
# set mc parameters
mc_ser = 1
mc_mea = 0
n_mcarlo = 10
n_trials = 10
sim_mcarlo.use_ncores = 5
not_conv_ser = []

# set simulation parameters
mc_settings = SimSettings(con_pkg_path)
mc_settings.time_init = 0
mc_settings.time_final = 120
mc_settings.time_step = 0.2
mc_settings.ivp_precision = 0.03

# set Monte Carlo functions
sim_mcarlo.function_generate = taes_random_fixed_error
sim_mcarlo.function_anl = basic_trf_anl
sim_mcarlo.function_setup = setup_model

# set Monte Carlo arguments
atderr_low_deg = 1
atderr_high_deg = 180 
atd_maxerror_deg_list = np.linspace(atderr_low_deg, atderr_high_deg, n_mcarlo).tolist()
anv_err_deg = np.rad2deg(np.pi/18)
sim_mcarlo.args_settings = mc_settings



for imcarlo in range(n_mcarlo):
    # clean existing results
    FileManager().clear_series_data(mc_settings.get_path_output(), mc_ser+imcarlo)
    
    # define ids
    id_list = [SimID(mc_ser+imcarlo, tru_id, mc_mea) for tru_id in range(n_trials)]

    # set error arguments
    sim_mcarlo.args_generate = [atd_maxerror_deg_list[imcarlo], anv_err_deg]

    # execute Monte Carlo trials
    gui.write_section('Running Simulations')
    sim_mcarlo.run_mc(id_list)    

    # check convergence at specified instants
    gui.write_section('Checking Convergence')
    mc_conv_lim = np.pi/360
    times = [mc_settings.time_final]
    conv_percent = {}

    for t in times:
        not_conv_list = get_not_converged(ObsModelKeys.atd_err_ang_norm, (0,0), t, id_list, mc_conv_lim, con_pkg_path)
        conv_percent[t] = 100 * (len(id_list) - len(not_conv_list)) / len(id_list)

    # save percentage of conveged trials
    not_conv_tru = [simid.tru for simid in not_conv_list]
    aux_fman = FileManager()
    aux_fman.save_csv_dict(mc_settings.get_path_output(), SimID(mc_ser+imcarlo), conv_percent, ObsModelKeys.percent_converged_trials, aux_fman.KEY_TAG_ANL)
    aux_fman.save_csv_dict(mc_settings.get_path_output(), SimID(mc_ser+imcarlo), {ObsModelKeys.not_converged_trials : not_conv_tru}, ObsModelKeys.not_converged_trials, aux_fman.KEY_TAG_ANL)

    # compute mean error
    gui.write_section('Computing Mean')
    compute_converged_mean(mc_ser+imcarlo, mc_mea, ObsModelKeys.atd_err_ang_norm, ObsModelKeys.mean_conv_atd_err_ang_norm, con_pkg_path, not_conv_list)
    compute_converged_mean(mc_ser+imcarlo, mc_mea, ObsModelKeys.anv_err_norm, ObsModelKeys.mean_conv_anv_err_norm, con_pkg_path, not_conv_list)
    
    if conv_percent[times[-1]] < 10**-7:
        not_conv_ser.append(mc_ser+imcarlo)

# plot results
gui.end_msg('Plotting Results')
conv_data = {
    'Error' : [], 'Percentage converged at 300s' : []
}
acell = AnlCell(con_pkg_path)
plt.figure(1)
for imcarlo in range(n_mcarlo):
    # load mean error
    acell.id = SimID(mc_ser+imcarlo)
    acell.data.anl.update(**acell.load_results([ObsModelKeys.percent_converged_trials]))
    if imcarlo+mc_ser not in not_conv_ser:
        acell.data.anl.update(**acell.load_results([ObsModelKeys.mean_conv_atd_err_ang_norm]))
        # save
        tarr, darr = acell.data.anl[ObsModelKeys.mean_conv_atd_err_ang_norm].get_plot_arrays((0,0))
        plot_obj = {'time' : tarr, str(mc_ser+imcarlo) : darr[0]}
        plt.plot('time', str(mc_ser+imcarlo), '', data=plot_obj)
    
    conv_data['Error'] += [atd_maxerror_deg_list[imcarlo]]
    conv_data['Percentage converged at 300s'] += [acell.data.anl[ObsModelKeys.percent_converged_trials][str(times[0])]]
    
plt.xlabel('Time')
plt.ylabel('Attitude Error')
plt.legend()
plt.savefig(Path(mc_settings.get_path_output(), 'convergence-error-anl.png'))

plt.figure(2)
plt.plot('Error', 'Percentage converged at 300s', data=conv_data, marker='o')
plt.xlabel('Initial Error')
plt.ylabel('Percentage Converged')
plt.savefig(Path(mc_settings.get_path_output(), 'convergence-percentage-anl.png'))
    
gui.end_msg('Finished Convergence Analysis')
