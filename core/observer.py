''' Manage estimation values and methods for a given simulation.

    Classes
    -------
    Observer
        Manage estimation schemes of a simulation.
    ObsKeys
        Organize observer specific keys.

'''
from copy import deepcopy
from typing import Tuple
from for_atd_sim.lib_data import DataSeries
from for_atd_sim.lib_data import DataPoint

class Observer():
    ''' Manage estimation schemes of a simulation.

        Instance Variables
        ------------------
            obs_init : dict
                Initial observer state.
            obs_state : dict
                Current observer state.
            obs_ser : dict
                Observer data series.
            obs_params : dict
                Parameters of the observer.
            obs_model : callable
                Observer model.
            setup_fun : callable
                Observer setup function.
    '''

    def __init__(self):
        ''' Create observer instance. '''
        self.obs_init = {}
        self.obs_state : dict[str, DataPoint]= {}
        self.obs_par = {}
        self.obs_model = callable
        self.setup_fun = callable
        self.det_mode = False
        self.other_data : dict[str, list] = {}


    def new_estimate(self, mea_data : dict[str, DataPoint]) -> Tuple[dict[str, DataPoint],dict[str, DataPoint]]:
        ''' State obs_model. Interface for different methods.

            Parameters
            ----------
            TODO
        '''
        new_state, other = self.obs_model(mea_data, self.obs_state, self.obs_par)

        # save current state
        self.obs_state.update(**new_state)

        return new_state, other


    def compute_estimates(self, sensor_data : dict[str, DataSeries]) -> dict[str, DataSeries]:
        ''' Get estimates from sensor data. '''
        
        obs_data : dict[str, DataSeries] = {}
        other_data : dict[str, DataSeries] = {}

        len_vec = [len(v.series) for v in sensor_data.values()]


        first = 0
        if not self.det_mode:
            first = 1
            for k in self.obs_state:
                obs_data[k] = DataSeries()
                obs_data[k].series.append(deepcopy(self.obs_state[k]))


        # estimate by time
        for i in range(first, min(len_vec)):
            
            mea_idata = {ikey : sensor_data[ikey].series[i] for ikey in sensor_data}

            est_idata, iother = self.new_estimate(mea_idata)
            
            # initialize data series in first iteration
            if obs_data == {}:
                for k in est_idata:
                    obs_data[k] = DataSeries()
            if other_data == {}:
                for k in iother:
                    other_data[k] = DataSeries()

            for k in est_idata:
                obs_data[k].series.append(deepcopy(est_idata[k]))
            for k in iother:
                other_data[k].series.append(deepcopy(iother[k]))

        self.other_data = other_data

        return obs_data


        # # get length (from sensor data)
        # for ikey in sensor_data:
        #     sim_len = len(sensor_data[ikey].series)
        #     break

        # # append initial estimates
        # for ikey in self.obs_init:
        #     if ikey not in self.obs_ser:
        #         self.obs_ser[ikey] = dlib.DataSeries()

        #     self.obs_ser[ikey].series.append(
        #         copy.deepcopy(self.obs_init[ikey])
        #     )

        # # get estimates for each time interval

        # # estimate zero if it is determination
        # obs_ini = 1
        # if self.det_mode:
        #     obs_ini = 0
            
        # # measurment at zero not used
        # for i in range(obs_ini, sim_len):

        #     # reinitialize measurement
        #     imea = {}

        #     # parse measurement
        #     for ikey in sensor_data:
        #         imea.update({ikey : sensor_data[ikey].series[i]})

        #     # get estimate
        #     self.estimate(imea, **kwargs)

        #     # append to data series
        #     for ikey in self.obs_state:
        #         if ikey not in self.obs_ser:
        #             self.obs_ser[ikey] = dlib.DataSeries()

        #         self.obs_ser[ikey].series.append(
        #             copy.deepcopy(self.obs_state[ikey])
        #         )

        # return self.obs_ser
