''' Initialize, setup, and manage simulation tasks.

'''
from pathlib import Path
from typing import List, Union
import numpy as np

# core classes
import for_atd_sim.core.keys as km
from for_atd_sim.core.simsettings import SimSettings
from for_atd_sim.core.simid import SimID
from for_atd_sim.core.simdata import SimData
from for_atd_sim.core.simmodel import SimModel 

# io classes
import for_atd_sim.lib_io.file_manager as fm

# TODO : delete
MAX_TIME_CONV = 2000
KEY_SETT = 'settings'
KEY_PARAM = 'params'

class SimCell:
    ''' Manage a single simulation task. '''

    def __init__(self, path_data : Path):
        ''' Initialize and setup simulation from input.

            Input method is given in settings file.
        '''
        self.settings = SimSettings(path_data)
        self.fileman = fm.FileManager()
        self.model = SimModel()
        self.id = SimID()
        self.data = SimData()


    def update_settings(self, input_settings : Union[dict,SimSettings]):
        ''' Update settings from file and arguments.
        
            Priority:
            1. Arguments
            2. File
            3. Default       
        '''
        # load settings file
        file_settings = fm.load_md_dict(self.settings.get_path_settings())

        # update file settings with arguments
        if isinstance(input_settings, SimSettings):
            input_settings = input_settings.get_custom_options_dict()
        file_settings.update(input_settings)
        
        # update settings
        self.settings.update_settings(**file_settings)
       

    def update_input(self, input_data : dict) -> None:
        ''' Update input data from file and arguments. 

            Priority:
            1. Arguments
            2. File
        '''        
        # get input data path
        input_path = self.settings.get_path_input()

        # load data from file 
        finput_data = self.fileman.load_file_data(input_path)

        # update data from argument 
        finput_data.update(input_data)

        # save initial data
        self.data.ini = finput_data


    def load_results(self, key_list : List[str]) -> dict:
        ''' Load and update results from file.'''
        load_data = {}
        for ikey in key_list:
                idata = self.fileman.load_single_file(self.settings.get_path_output(), self.id, ikey)                
                load_data.update({ikey : idata})
                
        return load_data
    

    def save_settings(self):
        self.fileman.save_csv_dict(self.settings.get_path_output(), SimID(self.id.ser), self.settings.get_custom_options_dict(), KEY_SETT)


    def save_parameters(self):
        self.fileman.save_csv_params(self.settings.get_path_output(), SimID(self.id.ser, self.id.tru), self.data.ini, KEY_PARAM)


    def save_results(self, key_list : List[str]) -> None:
        ''' Saves results in files. '''
        for ikey in key_list:
            self.fileman.save_csv_files(
                self.settings.get_path_output(),
                self.id,
                {ikey : self.data.res[ikey]}
            )
        

    def get_true_values(self, load=False) -> None:
        ''' Loads or generates problem true values. '''

        if load:
            true_data = self.load_results(self.model.pro_keys)
            self.data.update_results(true_data)
        else:
            true_data = self.model.problem.generate_true_values(self.get_time_vector())
            self.data.update_results(true_data)
            self.save_results(self.model.pro_keys)


    def get_measurements(self, load=False) -> None:
        ''' Loads or generates measurements. '''
        
        if load:
            mea_data = self.load_results(self.model.sen_keys + self.model.sen_cov_keys)
            self.data.update_results(mea_data)
        else:
            pro_data = self.data.get_specific_data(self.model.pro_keys)
            mea_data = self.model.sensors.generate_measurements(pro_data)
            self.data.update_results(mea_data)
            self.save_results(mea_data)



    def get_estimates(self, load=False):
        ''' Estimate solution for the problem. '''

        if load:
            obs_data = self.load_results(self.model.obs_keys)
            self.data.update_results(obs_data)
        else: 
            mea_data = self.data.get_specific_data(self.model.sen_keys + self.model.sen_cov_keys)
            obs_data = self.model.observer.compute_estimates(mea_data)
            self.data.update_results(obs_data)
            self.save_results(obs_data)


    def get_time_vector(self) -> np.ndarray:
        ''' Returns time vector from settings.'''

        time_i = self.settings.time_init
        time_f = self.settings.time_final
        d_time = self.settings.time_step

        return np.arange(time_i, time_f+d_time, d_time, float)


    def import_series_data(self, import_path : Path):
        self.fileman.clear_series_data(self.settings.get_path_output(), self.id.ser)
        import_files = self.fileman.find_import_series(import_path, self.id)
        self.fileman.import_files(import_files, self.settings.get_path_output(), self.id)        
        

    def load_import_settings(self):
        file_settings = self.fileman.load_single_file(self.settings.get_path_output(), SimID(self.id.ser), KEY_SETT)
        self.settings.update_settings(**file_settings)


    def load_parameters(self):
        input_parameters = self.fileman.load_single_file(self.settings.get_path_output(), SimID(self.id.ser, self.id.tru), KEY_PARAM)
        self.data.ini.update(**input_parameters)

