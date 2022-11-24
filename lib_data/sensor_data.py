''' Management classes and functions for sensors and measurements.

    Classes
    -------
    SensorPoint
        Data point for sensors and measurements.

'''
from typing import Tuple
from .basic_data import DataIndex
from .basic_data import DataPoint
from for_atd_sim.lib_hw.sensor import SensorModel


class SensorPoint(DataPoint):
    ''' Manage data point for a group of sensors.

        Instance Variables
        ------------------
        sensors


        Methods
        -------
        measure
            Save and return new measurements from sensors.

    '''
    
    def __init__(self):
        ''' Execute superclass constructor and add a sensor set. '''
        super().__init__()        
        self.sensors :  dict[DataIndex, SensorModel] = {}


    def measure(self, tru_pnt : DataPoint) -> Tuple[DataPoint,DataPoint]:
        ''' Returns the new measurement and covariance data points for each sensor.

            Arguments
            ---------
            tru_pnt :  true values data point

            Returns
            -------
            Tuple[DataPoint,DataPoint]

        '''
        mea_dp = DataPoint(tru_pnt.time)
        cov_dp = DataPoint(tru_pnt.time)
        
        # save new time
        self.time = tru_pnt.time

        # compute and save new measurements
        for (index, sensor) in self.sensors.items():                        
            mea_dp.data[index] = sensor.measure(tru_pnt.data[index])
            cov_dp.data[index] = sensor.cov

        # no sensor means it is a known reference 
        for index in tru_pnt.data:
            if index not in self.sensors:
                mea_dp.data[index] = tru_pnt.data[index]

        return mea_dp, cov_dp
