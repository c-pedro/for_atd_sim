''' Manage the true data.

    Classes
    -------
    Problem
        Manage true data.
'''
from typing import List
from numpy import ndarray
from for_atd_sim.lib_data.basic_data import DataPoint
from for_atd_sim.lib_data.basic_data import DataSeries

class Problem():
    ''' Manage true data model.

    '''
    def __init__(self):
        self.true_init : dict[str, DataPoint] = {} # initial groundtruth
        self.true_par = {} # model parameters
        self.true_model = callable
        # self.true_data = {}
        self.setup_fun = callable


    def generate_true_values(self, time_vec : List[ndarray]) -> dict[str, DataSeries]:
        ''' Generate and return true data. '''

        true_data = self.true_model(time_vec, self.true_init, self.true_par)

        return true_data
