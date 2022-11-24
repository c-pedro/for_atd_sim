''' Class that manages the measurement set and interfaces with the sensors.

    Includes a method to measure the values given by the ground truth.
'''
from copy import deepcopy
from for_atd_sim.lib_data import DataPoint
from for_atd_sim.lib_data import SensorPoint
from for_atd_sim.lib_data import DataSeries
from typing import Tuple
from typing import List


class Sensors():
    ''' Manages the measurement set and interfaces with sensors. '''

    def __init__(self):
        self.sensor_db: dict[str, SensorPoint] = {}
        self.sensor_par = {}
        self.sensor_ser = {}
        self.keymap: dict[str, List[str,str]] = {}
        self.setup_fun = callable

    def new_measurement(self, sen_key: str, true_data: DataPoint) -> Tuple[DataPoint, DataPoint]:
        ''' Updates measurement of each groundtruth variable with a sensor.

            Parameters
            ----------
            groundtruth : dict
                Groundtruth data.        
        '''
        return self.sensor_db[sen_key].measure(true_data)


    def generate_measurements(self, true_data: dict[str, DataSeries]) -> dict[str, DataSeries]:
        ''' Return measurement series dict.

            Parameters
            ----------
            true_data : dict[str, DataSeries]
                True data.

            Returns
            -------
            dict[str, DataSeries]
                Measurement data.
        '''
        sensor_data: dict[str, DataSeries] = {}

        # make measurements one series at a time
        for true_key, (sen_key, cov_key) in self.keymap.items():
            true_dser = true_data[true_key]
            measr_dser = DataSeries()
            covar_dser = DataSeries()

            for true_pnt in true_dser.series:
                mea, cov = self.new_measurement(sen_key, true_pnt)
                measr_dser.append_data(mea)
                covar_dser.append_data(cov)

            sensor_data.update({
                sen_key : deepcopy(measr_dser),
                cov_key : deepcopy(covar_dser)

            })

        return sensor_data
