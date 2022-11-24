from pathlib import Path
from pro_3hetfor import keys
from for_atd_sim.obs_detatd_inerel.keys import ObsModelKeys
from for_atd_sim.core.simcell import SimCell
from for_atd_sim.core.anlcell import AnlCell
from for_atd_sim.obs_detatd_inerel.model import setup_model


scell = SimCell(Path(__file__).parent)

scell.update_settings({})
scell.update_input({})
setup_model(scell.model, scell.data.ini, scell.settings)
scell.id.set_new_id(1,1,0)

scell.get_true_values()

# testing : load sim results
scell.data.res = {}
scell.get_true_values(True)

# testing : make measurements
scell.get_measurements()

# testing : load measurements
del scell.data.res[keys.ProModelKeys.anv]
scell.get_measurements(True)


# testing : estimate attitude
scell.get_estimates()

# testing : load esObsModelKeys
del scell.data.res[ObsModelKeys.atd]
scell.get_estimates(True)


acell = AnlCell(Path(__file__).parent)
acell.set_from_simcell(scell)

from for_atd_sim.lib_atd import trf_atd_error_angle_list
acell.transform_datalist([keys.ProKeys.atd, ObsModelKeys.atd], ObsModelKeys.atd_err_ang, trf_atd_error_angle_list)


# test : plot simulation results
kw_list = [
    {'linestyle' : ist}
    for ist in ['--',':','-.', '--',':','-.', '--',':','-.']
]
# acell.plot_var(keys.ProKeys.atd, (0,1), 'test_atd', kw_list)
# acell.plot_var(ObsModelKeys.atd, (0,1), 'test_atd_true', kw_list)
acell.plot_var(ObsModelKeys.atd_err_ang, (0,1), 'test_atd_ang_err', kw_list)




print('Passed')




# # pylint: disable=repeated-keyword

# import time
# import sys
# from pathlib import Path

# if not Path(__file__).parent.parent in sys.path:
#     sys.path.append(Path(__file__).parent.parent.parent.__str__())

# # import core modules
# from for_atd_sim.core.simcell import SimCell
# import for_atd_sim.core.simcell as cell
# import for_atd_sim.core.anlcell as acell

# # problem  and sensors
# import for_atd_sim.pro_3hetfor.problem as problem
# import for_atd_sim.pro_3hetfor.sensor_hub as sensors

# # subpackage modules
# import for_atd_sim.obs_detatd_inerel.keys as kpkg
# import for_atd_sim.obs_detatd_inerel.algorithm as alg
# import for_atd_sim.obs_detatd_inerel.commands as cmd
# import for_atd_sim.obs_detatd_inerel.observer as estimator

# # import for_atd_sim.lib_post.custom_scale


# if __name__ == "__main__":

#     print(
#         '\n* --- Numerical Analysis of Noise --- *\n'
#     )

#     # set number of trials
#     N_TRIALS = 2
#     OFFSET = 20

#     for n, n_file in enumerate(
#         [
#             'input-degen.md',
#             # 'input-coplanar.md',
#             # 'input-amb.md',
#         ]
#     ):

#         # get time
#         ser_time = time.time()
#         # feedback
#         print('\n* ------------------------------------- *')
#         print(' > Simulating input number {}'.format(n+1))

#         custom = {
#             # settings
#             kpkg.SettKeys.OPT_NOISE : True,
#             kpkg.SettKeys.OPT_CONFIG : kpkg.SettKeys.CONFIG_INPUT,
#             # kpkg.SettKeys.OPT_TIME_FINAL : 100,
#             kpkg.SettKeys.OPT_SERIES_ID : n + OFFSET,
#             acell.anl.Analyzer.CMD_LIST : cmd.cmd_list_mod,
#             kpkg.SettKeys.OPT_FNAME_INPUT : n_file,
#             # kpkg.SettKeys.OPT_FNAME_INPUT : 'input-degen.md',
#             # kpkg.SettKeys.OPT_FNAME_INPUT : 'input-reg.md',
#             # kpkg.SettKeys.OPT_FNAME_INPUT : 'input-amb.md',

#             # TODO : as option
#             alg.kComCov : True,

#             # setup functions
#             cell.KEY_PRO_SETUP_FUN : problem.setup_hetfor,
#             cell.KEY_SEN_SETUP_FUN : sensors.setup_sensors,
#             cell.KEY_OBS_SETUP_FUN : estimator.setup_observer,
#         }

#         # initialize  simulation cell
#         sim_cell = SimCell(__file__, **custom)

#         # setup settings
#         sim_cell.setup_settings(**custom)
#         sim_cell.settings.show_options()

#         # TODO as settings
#         # initialize stats keys
#         sim_cell.analyzer.stats.init_var(kpkg.ObsKeys.atd_err_ang)

#         # set system
#         sim_cell.load_input(**custom)

#         # set groundtruth id
#         sim_cell.id_man.grnID = 1

#         # generate groundtruth
#         print(' > Generating groundtruth')
#         sim_cell.gen_ground_data(**custom)


#         feed_time = time.time()
#         print(' > Running trials')
#         # run trials
#         for mid in range(N_TRIALS):

#             # if time.time() - feed_time > 2:
#             # feedback
#             sys.stdout.write(
#                 '\r > at trial number {} of {}'.format(mid, N_TRIALS)
#             )
#             feed_time = time.time()

#             # set id
#             sim_cell.id_man.meaID = mid

#             # generate measurements
#             sim_cell.gen_measurement_data(**custom)

#             # generate estimate attitude
#             sim_cell.estimate_data(**custom)

#             # analyze results
#             sim_cell.analyze(**custom)

#             # add to sum
#             sim_cell.analyzer.update_stats_sum()

#         # compute mean
#         sim_cell.analyzer.update_mean()
#         sim_cell.analyzer.update_rms()

#         # get metrics
#         sim_cell.analyzer.analyze(cmd.plot_special_metrics())

#         sys.stdout.write(
#             '\r > Computing statistics     \n'
#         )
#         # make trials
#         for mid in range(N_TRIALS):

#             # feedback
#             sys.stdout.write(
#                 '\r > at trial number {} of {}'.format(mid, N_TRIALS)
#             )
#             feed_time = time.time()

#             # set id
#             sim_cell.id_man.meaID = mid

#             # load results
#             sim_cell.analyzer.load_target_results(
#                 **{
#                     sim_cell.analyzer.KEY_OPT_FORCELOAD : True,
#                     sim_cell.analyzer.KEY_DATA : [
#                     [kpkg.ObsKeys.atd_err_ang],
#                     [kpkg.ObsKeys.atd_cov],
#                 ]}
#             )

#             # add to squared dev
#             sim_cell.analyzer.update_stats_sqdev()

#         # compute standard deviation
#         sim_cell.analyzer.update_sdev()

#         # get noiseless covariance
#         sys.stdout.write(
#             '\r > Computing noiseless theoretical value\n'
#         )
#         mid = N_TRIALS
#         custom.update({kpkg.SettKeys.OPT_NOISE : False})

#         sim_cell.setup_sensors(**custom)
#         # run noiseless
#         # set id
#         sim_cell.id_man.meaID = mid

#         # generate measurements
#         sim_cell.gen_measurement_data(**custom)

#         # generate estimate attitude
#         sim_cell.estimate_data(**custom)

#         print(' > Plotting and saving stats')
#         # post analysis
#         for idx in [(1,2), (1,3), (0,1)]:
#         # for idx in [(1,2), (1,3)]:
#             # get stats plot command
#             plot_cmd = cmd.plot_stats(idx)[0][1]
#             # plot stats
#             sim_cell.analyzer.plot_stats(**plot_cmd)

#         # save stats variables
#         sim_cell.analyzer.save_stats()

#         # feedback
#         ser_time = time.time() - ser_time

#         print('\n* --- Simulation Number {} Complete --- *'.format(n+1))
#         print('* --- Duration : {:.1f} seconds         --- *'.format(ser_time))
#         print('* ------------------------------------- *')

#     # show current plots
#     sim_cell.analyzer.show_plots()

#     # feedback
#     print('* ---------- Analysis Complete ------------ *')