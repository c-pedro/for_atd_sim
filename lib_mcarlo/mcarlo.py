''' Monte Carlo Simulation '''
import time
from multiprocessing import Pool
from multiprocessing import cpu_count
from typing import List 

# package imports
from for_atd_sim.core.simid import SimID
from for_atd_sim.core.simmodel import SimModel
from for_atd_sim.core.simsettings import SimSettings
from for_atd_sim.core.simcell import SimCell
from for_atd_sim.core.anlcell import AnlCell

import for_atd_sim.lib_gui.terminal as terminal


def empty_generate() -> dict:
    return {}

def empty_anl(acell : AnlCell):
    return

def empty_setup(smodel : SimModel, input_data : dict, settings : SimSettings):
    return


class MonteCarlo():
    ''' Manage a Monte Carlo simulation. '''

    def __init__(self, pkg_path):
        self.path_package = pkg_path
        self.use_ncores = 1
        self.function_generate = empty_generate
        self.args_generate = []
        self.function_anl = empty_anl
        self.args_settings = {}
        self.function_setup = empty_setup

        self.load_true = False
        self.load_meas = False
        self.load_pars = False
        self.load_sett = False


    def get_core_target(self, n_tasks : int, offset = 2) -> int:
        ''' Return number of cores to use. '''
        n_cores = cpu_count()

        use_cores = self.use_ncores

        if n_tasks < use_cores:
            return n_tasks
        
        if use_cores >= n_cores:
            use_cores = n_cores - offset
        
        self.use_ncores = use_cores
        return use_cores


    def simulate_trial(self, trial_id : SimID):

        terminal.write_bounded(str(trial_id))

        scell : SimCell = SimCell(self.path_package)
        acell : AnlCell = AnlCell(self.path_package)

        scell.id = trial_id
        
        if self.load_sett:
            scell.load_import_settings()
        else:
            scell.update_settings(self.args_settings)

        if self.load_pars:
            scell.load_parameters()
        else:    
            scell.update_input(self.function_generate(*self.args_generate))

        self.function_setup(scell.model, scell.data.ini, scell.settings)

        scell.save_settings()
        scell.save_parameters()

        scell.get_true_values(self.load_true)
        scell.get_measurements(self.load_meas)
        scell.get_estimates()

        acell.set_from_simcell(scell)

        self.function_anl(acell)


    def run_mc(self, id_list : List[SimID]):
        ''' Run a Monte Carlo set of simulations. '''

        # execution time
        exe_time = time.time()

        # TODO : gui feedback 
        terminal.init_msg('Monte Carlo Routine')

        # set number of processes
        n_process = self.get_core_target(len(id_list), 3)
        terminal.write_bounded('CORES USED : '+ str(n_process))

        terminal.write_task('Initializing Simulations')
        with Pool(n_process) as mc_pool:
            result = mc_pool.map_async(self.simulate_trial,  id_list)
            result.wait()

        terminal.write_exe_duration(time.time() - exe_time)
