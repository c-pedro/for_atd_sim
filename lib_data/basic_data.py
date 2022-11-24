''' Basic Data Structures

    Fundamental data structures common to all simulations.
    As broad and simple as possible.

    Hierarchy
    ---------
    1. Data Point
    2. Data Series

    Classes
    -------
    DataPoint
        Data relative to a time instant.
    DataSeries
        Data relative to a time interval. Several DataPoint instances.

'''
from typing import Tuple, List
import numpy as np


DataIndex = Tuple[int,int]
IndexedData = dict[DataIndex, np.ndarray]


class DataPoint():
    ''' Data relative to a time instant.

        Instance Variables
        ------------------
        time :  float
            Time instant of current data.
        data : IndexedData
            Data organized by index.

        Public Methods
        --------------
        validate_data
            Validates data stored in instance. (placeholder)

    '''
    def __init__(self, time=0.0):
        ''' Empty instance with time. '''
        self.time : float = time
        self.data : IndexedData = {}


    def validate_data(self):
        ''' Validates data stored in instance. (placeholder) '''
        assert True


    def has_converged(self, index, limit : float):
        ''' Returns true if all values below limit. '''
        if np.greater(self.data[index], limit):
            return False
        return True


class DataSeries():
    ''' Data relative to a time interval. 
    
        Contains list of multiple DataPoint instances.

        Instance Variables
        ------------------
        series : List[DataPoint]
            Array of DataPoints for a given quantity.

        Public Methods
        --------------
        append_data
            Appends DataPoint to series.
        get_data_array
            Return time and data entry as lists.
    
    '''
    def __init__(self):
        ''' Initialize instance with empty series. '''
        self.series : List[DataPoint] = []


    def append_data(self, data : DataPoint):
        ''' Appends DataPoint to series.

            Parameters
            ----------
            data : DataPoint
                Data to append to series.

        '''
        assert isinstance(data, DataPoint)
        self.series.append(data)


    def get_plot_arrays(self, index : DataIndex):
        ''' Returns the time and data arrays.'''
        time_arr : List[float] = []
        data_arr : List[List[float]]= []

        time_arr.append(float(self.series[0].time))
        # resize according to vector size
        for val in self.series[0].data[index].flatten():
            data_arr.append([val])

        for jdata in self.series[1:]:
            time_arr.append(float(jdata.time))

            for i, ival in enumerate(jdata.data[index].flatten()):
                data_arr[i].append(float(ival))

        return time_arr, data_arr
