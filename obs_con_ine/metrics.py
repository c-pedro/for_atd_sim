import for_atd_sim.obs_con_ine.keys as kpkg
from for_atd_sim.lib_data.basic_data import DataPoint
from for_atd_sim.lib_data.basic_data import DataSeries
from for_atd_sim.lib_post import metrics


def lemma1_metric_trf(**kwargs) -> DataSeries:
    ''' Return degnererate function series. '''
    metric_series = DataSeries()

    pro_ilos : DataSeries = kwargs[kpkg.kpro.ProModelKeys.ilos]
    pro_ref : DataSeries = kwargs[kpkg.kpro.ProModelKeys.ref]

    for i in range(len(pro_ilos.series)):
        score = DataPoint(pro_ilos.series[i].time)

        ilos_dp = pro_ilos.series[i]
        ref_dp = pro_ref.series[i]

        col_12 = metrics.get_vec_align_score(
            ref_dp.data[(2,0)], ilos_dp.data[(1,2)]
        )
        col_13 = metrics.get_vec_align_score(
            ref_dp.data[(3,0)], ilos_dp.data[(1,3)]
        )

        cop_2 = metrics.get_coplanar_proximity(
            ref_dp.data[(1,0)], ref_dp.data[(2,0)], ilos_dp.data[(1,2)])
        cop_3 = metrics.get_coplanar_proximity(
            ref_dp.data[(1,0)], ref_dp.data[(3,0)], ilos_dp.data[(1,3)])

        cop_branches = max(cop_2, cop_3)
    
        score.data[(0,0)] = min(col_12, col_13, cop_branches)
        
        metric_series.append_data(score)

    return metric_series