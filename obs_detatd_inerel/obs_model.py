''' Initialize, setup, and manage simulation for deterministic algorithm 3.
'''
from for_atd_sim.core.simmodel import SimModel
import for_atd_sim.obs_detatd_inerel.keys as kpkg
from for_atd_sim.obs_detatd_inerel.algorithm import compute_solution


def setup_observer(smodel : SimModel, input_data : dict) -> None:
    ''' Setup observer from input.

        Parameters
        ----------
        smodel : SimModel
            Simulation model instance.
        input_data : dict
            Observer initial state and parameters.
    '''
    # set mode
    smodel.observer.det_mode = True

    # set observer parameters
    smodel.observer.obs_par = {}

    # set keys for observer variables
    smodel.obs_keys = kpkg.obs_keys
    smodel.obs_cov_keys = kpkg.obs_cov_keys
    
    # set estimator module
    smodel.observer.obs_model = compute_solution
