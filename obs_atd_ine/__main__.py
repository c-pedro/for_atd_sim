''' Constrained Variational Observer
'''
from pathlib import Path
from pro_3hetfor import keys
from for_atd_sim.core.simcell import SimCell
from for_atd_sim.core.anlcell import AnlCell

from for_atd_sim.obs_atd_ine.model import setup_model
from for_atd_sim.obs_atd_ine.keys import ObsModelKeys

scell = SimCell(Path(__file__).parent)

scell.update_settings({})
scell.update_input({})

setup_model(scell.model, scell.data.ini, scell.settings)
scell.id.set_new_id(51,1,0)


# scell.import_series_data(Path(Path(__file__).parent.parent, 'obs_con_ine', 'output'))
# scell.load_import_settings()
# scell.load_parameters()


# scell.get_true_values(True)
scell.fileman.clear_series_data(scell.settings.get_path_output(), 50)
scell.get_true_values()


# scell.get_measurements(True)
scell.get_measurements()



scell.get_estimates()
# scell.get_estimates(True)


acell = AnlCell(Path(__file__).parent)
acell.set_from_simcell(scell)

from for_atd_sim.lib_atd import trf_atd_error_angle_list
acell.transform_datalist([keys.ProKeys.atd, ObsModelKeys.atd], ObsModelKeys.atd_err_ang, trf_atd_error_angle_list)


# test : plot simulation results
kw_list = [
    {'linestyle' : ist}
    for ist in ['--',':','-.', '--',':','-.', '--',':','-.']
]
acell.plot_var(keys.ProKeys.ilos, (1,2), 'test_ilos_true', kw_list)
acell.plot_var(keys.ProKeys.atd, (0,1), 'test_atd_true', kw_list)
acell.plot_var(ObsModelKeys.atd, (0,1), 'test_atd_obs', kw_list)
# acell.plot_var(ObsModelKeys.atd_err_ang, (0,1), 'test_atd_ang_err', kw_list, **{'yscale':'log'})

tarr1, darr1 = acell.data.anl[ObsModelKeys.atd_err_ang].get_plot_arrays((0,1))
tarr2, darr2 = acell.data.anl[ObsModelKeys.atd_err_ang].get_plot_arrays((0,2))
tarr3, darr3 = acell.data.anl[ObsModelKeys.atd_err_ang].get_plot_arrays((0,3))

acell.plot_data(
    [
        (tarr1, darr1[0], kw_list[0]), 
        (tarr2, darr2[0], kw_list[0]), 
        (tarr3, darr3[0], kw_list[0]), 
    ], 
    'test_atd_ang_err', **{'yscale':'log'})



acell.plot_var(keys.ProKeys.anv, (1,0), 'test_anv', kw_list)
acell.plot_var(ObsModelKeys.anv, (1,0), 'test_anv_obs', kw_list)
acell.plot_var(ObsModelKeys.anv_err, (1,0), 'test_anv_err', kw_list)

print('Passed Ine')
