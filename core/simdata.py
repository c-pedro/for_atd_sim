from for_atd_sim.lib_data.basic_data import DataPoint
from for_atd_sim.lib_data.basic_data import DataSeries
from typing import List

SingleData = dict[str, DataPoint]
MultiData = dict[str, DataSeries]


class SimData():    
    def __init__(self) -> None:
        self.ini : SingleData = {}
        self.res : MultiData = {}
        self.anl : MultiData = {}


    def update_results(self, new_data : MultiData):
        self.res.update(new_data)


    def get_specific_data(self, key_list : List[str]) -> MultiData:
        ''' Returns only true data '''
        return {k:v for (k,v) in (self.res|self.anl).items() if k in key_list}


    def get_data_list(self, key_list : List[str]) -> List[DataSeries]:
        ''' Returns only true data '''
        return [(self.res|self.anl)[k] for k in key_list if k in (self.res|self.anl)]

    
    def get_ini_dict(self) -> dict:
        ''' Returns dict with simulation parameters. '''
        init_dict = {}
        
        for ikey, idata in self.ini.items():
            init_dict[ikey] = idata
        
        return init_dict