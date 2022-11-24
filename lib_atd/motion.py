''' Formation motion models. '''
from typing import Tuple, List
from copy import deepcopy
import numpy as np
import for_atd_sim.core.keys as km
from for_atd_sim.lib_data import DataPoint
from for_atd_sim.lib_data import DataSeries
from for_atd_sim.lib_data import AttitudeData
from for_atd_sim.lib_data import AbsoluteDirectionData
from for_atd_sim.lib_data import RelativeDirectionData
from for_atd_sim.lib_data import InertialRelativeDirectionData
from for_atd_sim.lib_math import so3
import for_atd_sim.lib_math.random as rnd


LIM_MAN_ITIME = km.get_key('initial', 'time', 'maneuver')
LIM_MAN_FTIME = km.get_key('final', 'time', 'maneuver')
LIM_MAN_ANGLE = km.get_key('total', 'angle', 'maneuver')

STD_TIME_I = 0.0
STD_TIME_F = 1.0
STD_DANGLE = 0.0
STD_AXIS = np.array([[0],[0],[1]])


class Maneuver:
    ''' Manuever for a measurement. 
    
        Instance Variables
        ------------------
        time_i
            Initial instant of maneuver.
        time_f
            Final instant of maneuver.
        dangle
            Rotation angle rate.
        axis
            Rotation unit axis.
    
    '''
    KEY_ITIME = 'itime'
    KEY_FTIME = 'ftime'
    KEY_ANGLE = 'angle'
    KEY_AXIS = 'axis'
    KEY_INDEX = 'index'
    KEY_KEY = 'key'
    KEY_ANGRATE_MAX = 'maxanglerate'


    def __init__(self, *args, **kwargs) -> None:
        self.time_i = kwargs.get(self.KEY_ITIME , STD_TIME_I)
        self.time_f = kwargs.get(self.KEY_FTIME , STD_TIME_F)
        self.dangle = kwargs.get(self.KEY_ANGLE, STD_DANGLE)
        self.axis = kwargs.get(self.KEY_AXIS, STD_AXIS)

        # parse data from arguments
        if len(args) == 4:
            self.set_man_from_list(args)
        

    def is_active(self, time) -> bool:
        ''' Return True if maneuver is active. '''
        return bool(time >= self.time_i and time < self.time_f)


    def get_rotation(self, dtime):
        ''' Return the rotation applied to the prior vector. '''
        return so3.rot_mtx(self.dang * dtime, self.axis)


    def set_man_from_list(self, par_list : list):
        ''' Set parameters from list. '''
        self.time_i = par_list[0]
        self.time_f = par_list[1]
        self.dang = par_list[2]
        self.axis = par_list[3]



class ManeuverManager():
    ''' Maneuver database.'''
    def __init__(self) -> None:
        self.manv_db : dict[Tuple,List[Maneuver]] = {}


    def get_vector(self, time : float, dtime : float, vec_i, index):
        ''' Return vector transformed by manuever. '''
        # retrun samme vector when no maneuver exists 
        if index not in self.manv_db:
            return vec_i

        for man_i in self.manv_db[index]:
           if time > man_i.time_i  and time < man_i.time_f:
                vec_i = man_i.get_rotation(dtime) @ vec_i

        return vec_i


    def parse_manv(self, man_data : dict[Tuple,List[list]] ):
        for index, man_list in man_data.items():
            for iman in man_list:
                new_manv = Maneuver(*iman)
                if not index in self.manv_db:
                    self.manv_db[index] = []
                self.manv_db[index].append(new_manv)
            


def motion_formation(
    time : float,
    true_ini : dict,
    atd_pnt : AttitudeData,
    manv_mng : ManeuverManager,
    dtime : float
) -> dict:
    ''' Returns true observations for a given time instant. '''

    # parse variables
    ref_i : AbsoluteDirectionData = deepcopy(true_ini[km.ProKeys.ref])
    ilos_i : InertialRelativeDirectionData = deepcopy(true_ini[km.ProKeys.ilos])

    # initialize output
    ref_f = AbsoluteDirectionData()
    los_f = RelativeDirectionData()
    ilos_f = InertialRelativeDirectionData()

    # initialize output
    ref_f.time = time
    los_f.time = time
    ilos_f.time = time

    # update inertial reference vectors
    for index in ref_i.data:
        
        # consider vector in inertial frame
        if index[1] != 0:
            continue

        # update inertial reference
        ref_f.data[index] = manv_mng.get_vector(
            time,
            dtime,
            ref_i.data[index],
            index
        )

        # compute associated body reference
        ref_f.data[index[0], index[0]] = atd_pnt.get_atd(index) @ ref_f.data[index]

    # update inertial LOS vectors
    for index in [(1,2), (1,3)]: 
        
        # get transformed vector
        ilos_f.data[index] = manv_mng.get_vector(
            time,
            dtime,
            ilos_i.data[index],
            index,
        )

        # get inverse index
        inv_index = (index[1],index[0])

        # update opposing vector
        ilos_f.data[inv_index] = - ilos_f.data[index]

    # update body-fixed vectors
    for index in ilos_f.data:
        los_f.data[index] = atd_pnt.get_atd((index[0], 0)) @ ilos_f.data[index]

    # save changes
    out_db = {
        km.ProKeys.ref : ref_f,
        km.ProKeys.los : los_f,
        km.ProKeys.ilos : ilos_f,
    }

    return out_db


def parse_manv(true_par : dict[str,DataPoint]):
    ''' Returns maneuver manager from arguments. '''
    mnman = ManeuverManager()
    mnman.parse_manv(true_par[km.ProKeys.manv].data)

    return mnman


def get_formation_vectors(
    time_vec : list,
    true_init : dict,
    true_data : dict,
    true_par : dict
) -> dict:
    ''' Returns true vector observations in specific instants of time. 
    
        Parameters
        ----------

    '''
    ilos_ser = DataSeries()
    los_ser = DataSeries()
    ref_ser = DataSeries()
    
    # parse arguments
    atd_ser : DataSeries = true_data[km.ProKeys.atd]
    manv_mng = parse_manv(true_par)

    # assume fixed time step
    dtime = time_vec[1] - time_vec[0]

    for i, time in enumerate(time_vec):
        # current attitude
        atd_pnt = atd_ser.series[i]

        # compute reference
        if i==0:
            data_t = motion_formation(time, true_init, atd_pnt, manv_mng, dtime)
        else:
            data_t = motion_formation(time, data_t, atd_pnt, manv_mng, dtime)

        # update series
        ilos_ser.series.append(data_t[km.ProKeys.ilos])
        los_ser.series.append(data_t[km.ProKeys.los])
        ref_ser.series.append(data_t[km.ProKeys.ref])

    # update true observations
    true_data.update({
        km.ProKeys.ilos : ilos_ser,
        km.ProKeys.los : los_ser,
        km.ProKeys.ref : ref_ser
    })

    return true_data


def get_rnd_maneuver_aslist(index, **kwargs):
    ''' Returns a random maneuver encoded as list. '''

    # set default limits
    min_time = kwargs.get(LIM_MAN_ITIME, 0)
    max_time = kwargs.get(LIM_MAN_FTIME, 750)
    max_angle = kwargs.get(LIM_MAN_ANGLE, 2*np.pi/300)

    # get random values
    time_i = np.random.default_rng().random() * min_time
    time_f = max(time_i, np.random.default_rng().random() * max_time)
    angle = np.random.default_rng().random() * max_angle
    axis = rnd.random_unit_3d_vec()

    new_manv = {
        index : [
            time_i,     # initial time
            time_f,     # final time
            angle,      # total angle
            axis        # axis
        ]
    }

    return new_manv
