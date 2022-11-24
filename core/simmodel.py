from typing import List
from . import Problem
from . import Sensors
from . import Observer


class SimModel:
    ''' Manage s'''
    
    def __init__(self) -> None:
        self.problem = Problem()
        self.sensors = Sensors()
        self.observer = Observer()

        self.pro_keys : List[str] = []
        self.sen_keys : List[str] = []
        self.obs_keys : List[str] = []
        
        self.sen_cov_keys : List[str] = []
        self.obs_cov_keys : List[str] = []


    def setup_problem(self, **kwargs):
        ''' Call problem setup. '''
        self.problem.setup_fun(
            self.problem, self.data_input, self.settings, **kwargs
        )


    def setup_sensors(self, **kwargs):
        ''' Call sensor setup. '''
        self.sensor_hub.setup_fun(
            self.problem, self.sensor_hub,
            self.data_input,
            self.settings.options
        )


    def setup_observer(self, **kwargs):
        ''' Call observer setup. '''
        self.observer.setup_fun(
            self.problem,
            self.observer,
            self.data_input,
            **{**kwargs, **self.settings.options}
        )


    def setup_system(self, **kwargs):
        ''' Set parameters for entire simulation. '''

        # update settings with kwargs
        for skey in self.settings.options:
            if skey in kwargs:
                self.settings.options[skey] = kwargs[skey]

        # append setup
        if self.settings.options[self.OPT_APPEND]:
            # load prior trial
            self.load_trial(**kwargs)
            self.setup_append(**kwargs)

        # set problem
        self.setup_problem(**kwargs)
        # set sensor hub
        self.setup_sensors(**kwargs)
        # set observer
        self.setup_observer(**kwargs)