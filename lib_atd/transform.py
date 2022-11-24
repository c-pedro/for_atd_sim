''' Transformation functions
'''
from copy import deepcopy
import numpy as np
from for_atd_sim.lib_data.basic_data import DataPoint
from for_atd_sim.lib_data.basic_data import DataSeries
from for_atd_sim.lib_atd import compute_principal_angle
from for_atd_sim.lib_atd import compute_error_rotation
from for_atd_sim.lib_math import orthogonalize_mat


def trf_atd_error_list(tru_atd_ser : DataSeries, est_atd_ser : DataSeries):
    
    atd_err_ser = DataSeries()

    for i, itrue_atd in enumerate(tru_atd_ser.series):
        atd_err = DataPoint(itrue_atd.time)

        for index in itrue_atd.data:
            atd_err.data[index] = compute_error_rotation(itrue_atd.data[index], est_atd_ser.series[i].data[index])

        atd_err_ser.series.append(deepcopy(atd_err))
    
    return atd_err_ser


def trf_atd_error_angle_list(tru_atd_ser : DataSeries, est_atd_ser : DataSeries):
    
    angle_err_ser = DataSeries()
    atd_err_ser : DataSeries = trf_atd_error_list(tru_atd_ser, est_atd_ser)

    for i, ierr_atd in enumerate(atd_err_ser.series):
        angle_err = DataPoint(ierr_atd.time)

        for (index, rot) in ierr_atd.data.items():
            # try:
            angle_err.data[index] = np.array([compute_principal_angle(rot)])
            # except ValueError:
                # angle_err.data[index] = None # np.array([np.pi]) # TODO : check if it can cause load/save problems

        angle_err_ser.series.append(deepcopy(angle_err))
    
    return angle_err_ser


def trf_max_atd_error_angle_list(atd_err_angle_ser : DataSeries):
    
    max_angle_err_ser = DataSeries()

    for i, ierr_atd in enumerate(atd_err_angle_ser.series):
        max_angle_err = DataPoint(ierr_atd.time)

        ine_val = []
        for index in [(0,1), (0,2), (0,3)]: #ierr_atd.data.items():
            ine_val.append(ierr_atd.data[index])

        max_angle_err.data[(0,0)] = np.array([[max(ine_val)]])
        max_angle_err_ser.series.append(deepcopy(max_angle_err))
    
    return max_angle_err_ser


def trf_anv_error_norm_list(anv_err_ser : DataSeries):
    
    anv_err_norm_ser = DataSeries()

    for i, ierr_anv in enumerate(anv_err_ser.series):
        norm_err = DataPoint(ierr_anv.time)

        for index in ierr_anv.data: 
            norm_err.data[index] = np.linalg.norm(ierr_anv.data[index])
        
        anv_err_norm_ser.series.append(deepcopy(norm_err))
    
    return anv_err_norm_ser


def trf_max_anv_error_norm_list(anv_err_ser : DataSeries):
    max_anv_err_norm_ser = DataSeries()

    for i, ierr_anv in enumerate(anv_err_ser.series):
        max_norm_err = DataPoint(ierr_anv.time)
        ine_val = []

        for index in ierr_anv.data.items():
            ine_val.append(np.linalg.norm(ierr_anv.data[index]))

        max_norm_err.data[(0,0)] = np.array([[max(ine_val)]])
        max_anv_err_norm_ser.series.append(deepcopy(max_norm_err))
    
    return max_anv_err_norm_ser


def trf_orthogonalize_rotation_list(rot_ser : DataSeries):
    ortho_ser = DataSeries()

    for irotdp in rot_ser.series:
        ortho_dp = DataPoint(irotdp.time)

        for index, idx_rot in irotdp.data.items():
            ortho_dp.data[index] = orthogonalize_mat(idx_rot)
        
        ortho_ser.series.append(deepcopy(ortho_dp))
    
    return ortho_ser