from for_atd_sim.core.simmodel import SimModel
from for_atd_sim.core.simsettings import SimSettings
from for_atd_sim.pro_3hetfor.pro_model import setup_problem
from for_atd_sim.pro_3hetfor.sen_model import setup_sensors
from for_atd_sim.obs_detatd_inerel.obs_model import setup_observer


def setup_model(smodel : SimModel, input_data : dict, settings : SimSettings):
    
    setup_problem(smodel, input_data, settings)

    setup_sensors(smodel, input_data, settings)
    
    setup_observer(smodel, input_data)