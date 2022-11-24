'''
Lyapunov candidate functions.

'''
# import - math libraries
import numpy as np

# import - subpackage keys
import for_atd_sim.obs_con_ine.keys as kpkg

# import - data structures
from for_atd_sim.lib_data.basic_data import DataPoint
from for_atd_sim.lib_data.basic_data import DataSeries

# local keys
KEY_RESULTS = 'simulation_results'


def lyapunov_potential(time, pro_los, pro_ref, obs_atd, obs_anverr) -> DataPoint:
    '''
        Returns lyapunov value for a given data series index.

        Parameters
        ----------
        idx : int
            Index of series.

        Returns
        -------
        DataPoint
            Lyapunov value data point.
    '''
    # initialize data point
    aux_lya = DataPoint(time, np.zeros((3,3)))

    # initialize kinetic and potential energy values
    lya_kin = 0
    lya_pot = 0

    for index in [(1,1), (2,2), (3,3)]:
        # error kinetic energy
        lya_kin += 0.5 * obs_anverr[index[0],0].T @ obs_anverr[index[0],0]

        # vector reference potential
        lya_pot += float(
            0.5 * (
                (pro_ref[index] - obs_atd[0, index[0]].T @ pro_ref[index[0],0]).T
                @
                (pro_ref[index] - obs_atd[0, index[0]].T @ pro_ref[index[0],0])
            )
        )

    for index in [(1,2), (1,3)]:
        # vector relative constraints
        lya_pot += float(0.5 * (
            (pro_los[index]
                + obs_atd[0, 1].T @ obs_atd[0, index[1]] @ pro_los[index[1],index[0]]).T
            @
            (pro_los[index]
                + obs_atd[0, 1].T @ obs_atd[0, index[1]] @ pro_los[index[1],index[0]])
        ))

        # # scalar mixed (2 and 3)
        # lya_pot += float(
        #     0.5 * (
        #         pro_ref[1,1].T @ pro_los[index]
        #         + pro_ref[1,0].T @ obs_atd[0,index[1]] @ pro_los[index[1], index[0]]
        #     )**2
        # )

        # # scalar mixed (1)
        # lya_pot += float(
        #     0.5 * (
        #         pro_ref[index[1], index[1]].T @ pro_los[index[1],index[0]]
        #         + pro_ref[index[1],0].T @ obs_atd[0,1] @ pro_los[index]
        #     )**2
        # )

        # scalar refs (2 and 3)
        lya_pot += float(
            0.5 * (
                pro_ref[1,0].T @ pro_ref[index[1],0]
                - pro_ref[1,1].T @ obs_atd[0,1].T @ obs_atd[0,index[1]] @ pro_ref[index[1], index[1]]
            )**2
        )

    # # scalar relative constraints
    # lya_pot += float(
    #     0.5 * (
    #         pro_los[1,2].T @ pro_los[1,3]
    #         - pro_los[2,1].T @ obs_atd[0,2].T @ obs_atd[0, 3] @ pro_los[3,1]
    #     )**2
    # )

    # save value in data point
    aux_lya.data[0,0] = lya_kin + lya_pot
    aux_lya.data[0,1] = lya_kin
    aux_lya.data[0,2] = lya_pot
    aux_lya.compute_vld()

    return aux_lya


def lyapunov_trf(**kwargs) -> DataSeries:
    ''' Return Lyapunov function series. '''
    # parse data
    pro_los : DataSeries = kwargs[kpkg.ProKeys.los]
    pro_ref : DataSeries = kwargs[kpkg.ProKeys.ref]
    obs_atd : DataSeries = kwargs[kpkg.ObsKeys.atd]
    obs_anverr : DataSeries = kwargs[kpkg.ObsKeys.anv_err]

    # get series length
    ser_len = len(pro_los.series)

    # initialize Lyapunov data series
    lyap_series = DataSeries()

    for i in range(ser_len):

        # get time
        time = pro_los.series[i].time

        # append lyapunov data point
        lyap_series.append_data(lyapunov_potential(
            time=time,
            pro_los=pro_los.series[i].data,
            pro_ref=pro_ref.series[i].data,
            obs_atd=obs_atd.series[i].data,
            obs_anverr=obs_anverr.series[i].data
        ))

    return lyap_series
