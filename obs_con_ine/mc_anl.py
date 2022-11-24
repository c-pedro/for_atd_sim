''' Custom analysis functions '''
from pro_3hetfor import keys
from for_atd_sim.core.anlcell import AnlCell
from for_atd_sim.obs_con_ine.keys import ObsModelKeys
from for_atd_sim.lib_atd import trf_atd_error_angle_list
from for_atd_sim.lib_atd import trf_atd_error_angle_list
from for_atd_sim.lib_atd import trf_max_atd_error_angle_list
from for_atd_sim.lib_atd import trf_anv_error_norm_list
from for_atd_sim.obs_con_ine.metrics import lemma1_metric_trf


def custom_anl(acell : AnlCell):
    
    # transformations 
    acell.transform_datalist([keys.ProKeys.atd, ObsModelKeys.atd], ObsModelKeys.atd_err_ang, trf_atd_error_angle_list)
    acell.transform_datalist([ObsModelKeys.atd_err_ang], ObsModelKeys.atd_err_ang_norm, trf_max_atd_error_angle_list)
    acell.transform_datalist([ObsModelKeys.anv_err], ObsModelKeys.anv_err_norm, trf_anv_error_norm_list)
    acell.transform_datadict([keys.ProKeys.ref, keys.ProKeys.ilos], keys.ProKeys.metric_lem, lemma1_metric_trf)


def basic_trf_anl(acell : AnlCell):
    ''' Execute basic transformations on results.
    
        # transformations 
        1. Compute Attitude Error Principle Angle
        2. Compute Norm of Attitude Error Principle Angle
        3. Compute Norm of Angular Velocity Error
        4. Compute Proximity to Conditions of Lemma 1
    '''
    acell.transform_datalist([keys.ProKeys.atd, ObsModelKeys.atd], ObsModelKeys.atd_err_ang, trf_atd_error_angle_list)
    acell.transform_datalist([ObsModelKeys.atd_err_ang], ObsModelKeys.atd_err_ang_norm, trf_max_atd_error_angle_list)
    acell.transform_datalist([ObsModelKeys.anv_err], ObsModelKeys.anv_err_norm, trf_anv_error_norm_list)
    acell.transform_datadict([keys.ProKeys.ref, keys.ProKeys.ilos], keys.ProKeys.metric_lem, lemma1_metric_trf)


def basic_det_trf_anl(acell : AnlCell):
    ''' Execute basic transformations on results.
    
        # transformations 
        1. Compute Attitude Error Principle Angle
        2. Compute Norm of Attitude Error Principle Angle
    '''
    acell.transform_datalist([keys.ProKeys.atd, ObsModelKeys.atd], ObsModelKeys.atd_err_ang, trf_atd_error_angle_list)
    acell.transform_datalist([ObsModelKeys.atd_err_ang], ObsModelKeys.atd_err_ang_norm, trf_max_atd_error_angle_list)


def custom_plot_anl(acell : AnlCell):
    empty_3dict = {i:{} for i in range(3)}
    empty_1dict = {0:{}}
    
    # plot ilos + iref
    acell.plot_multivar(
        [
            (keys.ProModelKeys.ilos, (1,2), empty_3dict),
            (keys.ProModelKeys.ilos, (1,3), empty_3dict),
            (keys.ProModelKeys.ref, (1,0), empty_3dict),
            (keys.ProModelKeys.ref, (2,0), empty_3dict),
            (keys.ProModelKeys.ref, (3,0), empty_3dict),
        ],
        acell.fileman.name_man.get_file_id_str(acell.id) + 'vecs', 
        **{
            'yscale':'linear',
            'legend': ['i12', 'i13', 'i1', 'i2', 'i3'],
        }
    )

    # plot anv error
    anvobs_name = acell.fileman.name_man.get_filename(acell.id, 'anl', ObsModelKeys.anv_err_norm)
    acell.plot_multivar(
        [
            (ObsModelKeys.anv_err_norm, (1,0), empty_3dict),
            (ObsModelKeys.anv_err_norm, (2,0), empty_3dict),
            (ObsModelKeys.anv_err_norm, (3,0), empty_3dict),
            (keys.ProModelKeys.metric_lem, (0,0), empty_1dict),
        ],
        anvobs_name, 
        **{
            'yscale':'log',
            'legend': ['$\|\phi\|_\infty$', 'Lemma 1 Score' ],
        }
    )


    # plot : error norm + lemma 1
    atdobs_name = acell.fileman.name_man.get_filename(acell.id, 'anl', ObsModelKeys.atd_err_ang)
    acell.plot_multivar(
        [
            (ObsModelKeys.atd_err_ang_norm, (0,0), empty_1dict),
            (keys.ProModelKeys.metric_lem, (0,0), empty_1dict),
        ],
        atdobs_name, 
        **{
            'yscale':'log',
            'legend': ['$\|\|\epsilon\|\|_\infty$', 'Lemma 1 Score' ],
        }
    )



if __name__=='__main__':
    from pathlib import Path
    from for_atd_sim.core.simid import SimID
    from for_atd_sim.core.simcell import SimCell
    myanlcell = AnlCell(Path(__file__).parent)


    for i in range(10):

        myanlcell.id= SimID(401,i,0)
        # TODO : detect which are load required
        res = myanlcell.load_results([keys.ProModelKeys.ilos, keys.ProModelKeys.ref, ObsModelKeys.atd_err_ang_norm, ObsModelKeys.anv_err_norm, keys.ProModelKeys.metric_lem ])
        myanlcell.data.res.update(**res)
        custom_plot_anl(myanlcell)

