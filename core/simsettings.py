''' Manage settings for a simulation.

    Classes
    -------
    SimSettings
        Structure and manage input, simulation, and output options.
'''
from pathlib import Path
import for_atd_sim.lib_gui.terminal as trm


STD_DIR_INPUT = 'input'
STD_DIR_OUTPUT = 'output'
STD_FNAME_SETTINGS = 'settings.md'
STD_FNAME_INPUT = 'input.md'




class SimSettings:
    ''' Structure and manage input, simulation, and output options.

        Instance Variables
        ------------------
        options
            Option dictionary.
        path_data
            Current subapackage path.

    '''
    def __init__(self, path_data : Path):
        ''' Initialize settings with options as kwargs. '''

        self.set_default_options()

        # set subpackage path
        self.path_data = path_data


    def set_default_options(self):
        ''' Set the values of settings to default values. '''
        
        self.time_init = 0.0
        self.time_final = 60.0
        self.time_step = 0.5
        self.time_append = 25

        self.ivp_precision = 0.01 

        # file operations
        self.clear_results = True
        self.clear_figs = True
        self.load_data = True
        
        # directory names
        self.dir_input = STD_DIR_INPUT
        self.dir_output = STD_DIR_OUTPUT
        
        # file names
        self.fname_settings = STD_FNAME_SETTINGS
        self.fname_input = STD_FNAME_INPUT
        
        # file name tags # TODO : used or remove
        self.ftag_results = 'sim'
        
        # simulation options
        self.sim_noise = True
        self.sim_continuous = False
        self.sim_append = False
        self.sim_converge = False


    def show_options(self, serid : int):
        ''' Display main options in console. '''

        trm.write_task('Options')
        trm.write_bounded(
            'Series ID: {}'.format(serid),
            aln='<'
        )
        trm.write_bounded('Duration : {:^4.0f} seconds'.format(
            float(self.time_final),
            aln='<'
        ))
        trm.write_bounded(
            'Time Step : {:^5.3f} seconds'.format(
            float(self.time_step)),
            aln='<'
        )
        trm.write_bounded(
            'IVP Step : {:^5.3f} seconds'.format(
            float(self.ivp_precision)),
            aln='<'
        )
        trm.write_bounded(
            'Noise: {}'.format(
            self.sim_noise),
            aln='<'
        )

        if self.dir_input != STD_DIR_INPUT:
            trm.write_bounded(
                'Input Directory : {}'.format(self.dir_input),
                aln='<'
            )
        if self.dir_output != STD_DIR_OUTPUT:
            trm.write_bounded(
                'Output Directory : {}'.format(
                    self.dir_output),
                aln='<'
            )
        if self.fname_settings != STD_FNAME_SETTINGS:
            trm.write_bounded(
                'Settings File : {}'.format(self.fname_settings),
                aln='<'
            )
        if self.fname_input != STD_FNAME_INPUT:
            trm.write_bounded(
                'Input File : {}'.format(
                self.fname_input),
                aln='<'
            )
       

    def update_settings(self, **kwargs):

        # convert data type 
        for key in kwargs.keys():
            if not hasattr(self, key):
                continue

            data_type = type(getattr(self, key))
            if data_type != type(kwargs[key]):
                kwargs[key] = data_type(kwargs[key])

        # update 
        for item in kwargs.items():
            setattr(self, *item)


    def get_path_settings(self):
        return Path(self.path_data, self.dir_input, self.fname_settings)


    def get_path_input(self):
        return Path(self.path_data, self.dir_input, self.fname_input)


    def get_path_output(self):
        return Path(self.path_data, self.dir_output)


    def get_options_dict(self) -> dict:
        opt_dict = {**self.__dict__}
        del opt_dict['path_data']
        return opt_dict

    
    def get_custom_options_dict(self) -> dict:
        std_settings = SimSettings(Path())
        std_dict = std_settings.get_options_dict()
        
        custom_dict = {**self.__dict__}
        del custom_dict['path_data']

        for key, value in std_dict.items():
            if custom_dict[key] == value:
                del custom_dict[key]

        return custom_dict

