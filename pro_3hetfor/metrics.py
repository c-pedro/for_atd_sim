''' Functions for assessing degenerate and ambiguous metrics.

'''
# import - math libraries
import numpy as np

# import - subpackage keys
import for_atd_sim.obs_con_ine.keys as kpkg

# import - data structures
from for_atd_sim.lib_data.basic_data import DataPoint
from for_atd_sim.lib_data.basic_data import DataSeries

from for_atd_sim.lib_post import metrics
from for_atd_sim.lib_math.trigonometry import interval_pm_pi


# initialize legend array
leg_arr = {}
leg_arr[(1,2)] = 'degen 1-2'
leg_arr[(1,3)] = 'degen 1-3'
leg_arr[(1,12)] = 'degen 1-12'
leg_arr[(1,13)] = 'degen 1-13'
leg_arr[(2,12)] = 'degen 2-12'
leg_arr[(3,13)] = 'degen 3-13'
leg_arr[(0,0)] = 'ambiguous'


def degenerate_score(ilos_dp : DataPoint, ref_dp : DataPoint) -> DataPoint:
    ''' Return data point with degenerate score of data point.

        Parameters
        ----------
        ilos_data : DataPoint
            Current inertial LOS measurements datapoint.
        ref_data : DataPoint
            Current inertial reference measurements datapoint.

        Returns
        -------
        DataPoint
            Current degenerate scores.
    '''
    degen_score = DataPoint(ilos_dp.time)

    ilos_data = ilos_dp.data
    ref_data = ref_dp.data

    # setup degenerate comparison pairs
    deg_tuple_db = {}
    deg_tuple_db[(1,2)] = (ref_data[(1,0)], ref_data[(2,0)])
    deg_tuple_db[(1,3)] = (ref_data[(1,0)], ref_data[(3,0)]) 
    deg_tuple_db[(1,12)] = (ref_data[(1,0)], ilos_data[(1,2)]) 
    deg_tuple_db[(1,13)] = (ref_data[(1,0)], ilos_data[(1,3)]) 
    deg_tuple_db[(2,12)] = (ref_data[(2,0)], ilos_data[(1,2)]) 
    deg_tuple_db[(3,13)] = (ref_data[(3,0)], ilos_data[(1,3)]) 

    for index, degen_case in deg_tuple_db.items():
        aux_data = metrics.get_vec_align_score(
            degen_case[0],
            degen_case[1]
        )

        # save degenerate score
        degen_score.data[index] = aux_data

    return degen_score


def ambiguous_score(ilos_dp : DataPoint, ref_dp : DataPoint) -> DataPoint:
    ''' Return data point with ambiguous score of data point.

        Parameters
        ----------
        ilos_data : DataPoint
            Current inertial LOS measurements DataPoint.
        ref_data : DataPoint
            Current inertial reference measurements DataPoint.

        Returns
        -------
        DataPoint
            Current ambiguous scores.
    '''
    amb_score = DataPoint(ilos_dp.time)

    ilos_data = ilos_dp.data
    ref_data = ref_dp.data

    # compute both alphas
    alpha1 = metrics.get_alpha(
        ilos_data[(1,2)],
        ref_data[(1,0)],
        ilos_data[(1,3)]
    )
    alpha2 = metrics.get_alpha(
        ref_data[(2,0)],
        ref_data[(1,0)],
        ref_data[(3,0)]
    )

    # save smallest of both possible scores
    amb_score.data[(0,0)] = min(
        [
            abs(interval_pm_pi(alpha1 - alpha2)),
            abs(interval_pm_pi(alpha1 - alpha2 - np.pi))
        ]
    ) / np.pi

    return amb_score


def coplanar_score(ilos_dp : DataPoint, ref_dp : DataPoint) -> DataPoint:
    ''' Returns data point with coplanar branch scores.

        Parameters
        ----------
        time : float
            Current time.
        ilos_data : DataPoint
            Current inertial LOS measurements DataPoint.
        ref_data : DataPoint
            Current inertial reference measurements DataPoint.

        Returns
        -------
        DataPoint
            Current coplanar branch scores.
    '''
    # initialize score data point
    cop_score = DataPoint(ilos_dp.time)

    ilos_data = ilos_dp.data
    ref_data = ref_dp.data

    # compute both alphas
    for i in [2,3]:
        cop_aux = metrics.get_coplanar_proximity(
            ref_data[(1,0)],
            ref_data[(2,0)],
            ilos_data[(1,2)]
        )

        # save coplanrity score
        cop_score.data[(i,0)] = cop_aux

    return cop_score


def degenerate_metric_trf(**kwargs) -> DataSeries:
    ''' Return degnererate scores DataSeries. '''
    metric_series = DataSeries()

    pro_ilos : DataSeries = kwargs[kpkg.kpro.ProModelKeys.ilos]
    pro_ref : DataSeries = kwargs[kpkg.kpro.ProModelKeys.ref]

    for i in range(len(pro_ilos.series)):

        ilos_dp = pro_ilos.series[i]
        ref_dp = pro_ref.series[i]

        metric_series.append_data(
            degenerate_score(ilos_dp=ilos_dp,ref_dp=ref_dp)
        )

    return metric_series


def ambiguous_metric_trf(**kwargs) -> DataSeries:
    ''' Return degnererate function series. '''
    metric_series = DataSeries()

    pro_ilos : DataSeries = kwargs[kpkg.kpro.ProModelKeys.ilos]
    pro_ref : DataSeries = kwargs[kpkg.kpro.ProModelKeys.ref]

    for i in range(len(pro_ilos.series)):

        ilos_dp = pro_ilos.series[i]
        ref_dp = pro_ref.series[i]

        metric_series.append_data(
            ambiguous_score(ilos_dp=ilos_dp,ref_dp=ref_dp)
        )

    return metric_series
