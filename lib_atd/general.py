''' Functions that compute the true values from a model of dynamics. '''

from numpy import ndarray
from for_atd_sim.lib_data import DataPoint
from .rigid_body import get_formation_dynamics
from .motion import get_formation_vectors

def model_dynamics_het(time_vec : ndarray, true_init : dict[str, DataPoint], true_par : dict) -> dict:
    ''' Returns true values of a three vehicle heterogeneous formation. '''

    # compute attitude and angular velcity
    true_data = get_formation_dynamics(time_vec, true_init, true_par)

    # compute vehicle and reference positions
    true_data = get_formation_vectors(time_vec, true_init, true_data, true_par)

    return true_data
