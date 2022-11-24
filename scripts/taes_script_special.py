''' Simulates Constraint based Observer in Special Cases

'''
import for_atd_sim.scripts.script_path
import numpy as np
from pathlib import Path

from pro_3hetfor import keys
from for_atd_sim.obs_con_ine.keys import ObsModelKeys

from for_atd_sim.core.simid import SimID
from for_atd_sim.core.simcell import SimCell
from for_atd_sim.core.anlcell import AnlCell

from for_atd_sim.obs_con_ine.model import setup_model
from for_atd_sim.pro_3hetfor.generate import random_bounded_anv
from for_atd_sim.pro_3hetfor.generate import random_degenerate_maneuver
from for_atd_sim.pro_3hetfor.taes_special import taes_degenerate_configuration
from for_atd_sim.pro_3hetfor.taes_special import taes_ambiguous_configuration

from for_atd_sim.lib_atd.transform import trf_atd_error_angle_list
from for_atd_sim.lib_atd.transform import trf_max_atd_error_angle_list
from for_atd_sim.obs_con_ine.metrics import lemma1_metric_trf
from for_atd_sim.obs_con_ine.generate import get_initial_obs_zeroanv

from for_atd_sim.lib_gui import terminal


def get_amb_input():
    # get degenerate configuration
    anv_bounded = random_bounded_anv(np.pi/18)
    amb_config = taes_ambiguous_configuration()

    ilos12man = random_degenerate_maneuver((1,2), [0,0], [300,300], [2*np.pi/300,2*np.pi/300], **{'axis' : amb_config[keys.ProModelKeys.ref][(1,0)]})
    ref2man = {(2,0) : ilos12man[(1,2)]}

    deg_maneuver = {keys.ProModelKeys.manv : {
            **ilos12man,
            **ref2man,
        }
    }

    return amb_config | deg_maneuver | anv_bounded


def get_degen_input(*degen_args):
    # get degenerate configuration
    anv_bounded = random_bounded_anv(np.pi/18)
    deg_config = taes_degenerate_configuration(*degen_args)
    deg_maneuver = {keys.ProModelKeys.manv : {
            **random_degenerate_maneuver((1,2), [0,0], [300,300], [2*np.pi/300,2*np.pi/300], **{'axis' : deg_config[keys.ProModelKeys.ref][(2,0)]}),
            # **random_degenerate_maneuver((2,0), [0,0], [300,300], [0,2*np.pi/300]),
        }
    }

    return deg_config | deg_maneuver | anv_bounded


def run_trial(trial_cell : SimCell, input_dict : dict):
    
    trial_cell.update_input(input_dict)
    setup_model(trial_cell.model, trial_cell.data.ini, trial_cell.settings)    
    # trial_cell.save_params()# TODO : maneuver save incorrect

    trial_cell.get_true_values()
    trial_cell.get_measurements()
    trial_cell.get_estimates()

    acell = AnlCell(Path(__file__).parent)
    acell.set_from_simcell(trial_cell)

    acell.transform_datalist([keys.ProKeys.atd, ObsModelKeys.atd], ObsModelKeys.atd_err_ang, trf_atd_error_angle_list)
    acell.transform_datalist([ObsModelKeys.atd_err_ang], ObsModelKeys.atd_err_ang_norm, trf_max_atd_error_angle_list)
    acell.transform_datadict([keys.ProKeys.ref, keys.ProKeys.ilos], keys.ProKeys.metric_lem, lemma1_metric_trf)
   
    atdobs_name = trial_cell.fileman.name_man.get_filename(trial_cell.id, 'anl', ObsModelKeys.atd_err_ang)

    acell.plot_multivar(
        [
            (ObsModelKeys.atd_err_ang_norm, (0,0), {0:{}}),
            (keys.ProModelKeys.metric_lem, (0,0), {0:{}}),
        ],
        atdobs_name, 
        **{
            'yscale':'log',
            'legend': ['$\|\|\epsilon\|\|_\infty$', 'Lemma 1 Score' ],
        }
    )



if __name__=='__main__':
    terminal.init_msg('Special cases')

    n_trials = 1

    scell = SimCell(Path(Path(__file__).parent.parent, 'obs_con_ine'))
    scell.id = SimID(300, 0, 0)

    terminal.write_bounded('Degenerate')
    for case_args in [[(1,0), (3,0)], [(1,0), (1,3)]]:
        terminal.write_bounded(str(case_args))

        scell.fileman.clear_series_data(scell.settings.get_path_output(), scell.id.ser)
        scell.update_settings({})
        scell.save_settings()

        for tru_id in range(n_trials):
            terminal.write_current_trial(tru_id+1, n_trials)
            
            scell.id.tru = tru_id

            input_data = get_degen_input(*case_args)

            max_err_deg = 90
            deg_error = {
                (0,1) : np.random.default_rng().uniform(0, max_err_deg),
                (0,2) : np.random.default_rng().uniform(0, max_err_deg),
                (0,3) : np.random.default_rng().uniform(0, max_err_deg),
            }
            deg_error[(0,np.random.default_rng().choice([1,2,3]))] = max_err_deg
            input_data = get_initial_obs_zeroanv(input_data, deg_error)

            run_trial(scell, input_data)


        # increment series id
        scell.id.ser += 1

    terminal.write_bounded('Ambiguous')
    scell.fileman.clear_series_data(scell.settings.get_path_output(), scell.id.ser)
    scell.update_settings({})
    scell.save_settings()

    for tru_id in range(n_trials):
        terminal.write_current_trial(tru_id+1, n_trials)

        scell.id.tru = tru_id
        input_data = get_amb_input()
        max_err_deg = 90
        deg_error = {
            (0,1) : np.random.default_rng().uniform(0, max_err_deg),
            (0,2) : np.random.default_rng().uniform(0, max_err_deg),
            (0,3) : np.random.default_rng().uniform(0, max_err_deg),
        }
        deg_error[(0,np.random.default_rng().choice([1,2,3]))] = max_err_deg
        input_data = get_initial_obs_zeroanv(input_data, deg_error)

        run_trial(scell, input_data)


    terminal.end_msg('All Simulations Finished')