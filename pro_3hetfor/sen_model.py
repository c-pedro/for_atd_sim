''' Functions for setting the sesnors.

'''
# subpackage specific
import for_atd_sim.pro_3hetfor.keys as kpkg

# simulation - sensors
import for_atd_sim.core.sensors as senhub
from for_atd_sim.lib_hw import RateGyroDiscrete
from for_atd_sim.lib_hw import FocalPlane

from for_atd_sim.core.simmodel import SimModel
from for_atd_sim.core.simsettings import SimSettings


def setup_sensors(
        smodel : SimModel,
        input_data : dict,
        settings : SimSettings
) -> None:
    ''' Setup sensor hub from input.

        Parameters
        ----------
        sim_problem : Problem
            Problem instance.
        sim_hub : Sensors
            Sensor hub instance.
        input_params : dict
            Sensor properties.
        options : dict
            Settings options
    '''
    sen_anv = senhub.SensorPoint()
    sen_los = senhub.SensorPoint()
    sen_ref = senhub.SensorPoint()

    # set noise value
    noise = 0.0
    if settings.sim_noise:
        noise = 1.0

    for (index, value) in input_data[kpkg.SenKeys.anv_sdev].items():
        sen_anv.sensors[index] = RateGyroDiscrete(sdev_scale=value*noise)
        sen_anv.sensors[index].time_step = settings.time_step

    for (index, value) in input_data[kpkg.SenKeys.los_sdev].items():
        sen_los.sensors[index] = FocalPlane(sdev=value*noise)

    for (index, value) in input_data[kpkg.SenKeys.ref_sdev].items():
        sen_ref.sensors[index] = FocalPlane(sdev=value*noise)

    # set sensor parameters
    smodel.sensors.sensor_par = {
        kpkg.SenKeys.los_sdev : input_data[kpkg.SenKeys.los_sdev],
        kpkg.SenKeys.ref_sdev : input_data[kpkg.SenKeys.ref_sdev],
        kpkg.SenKeys.anv_sdev : input_data[kpkg.SenKeys.anv_sdev]
    }

    # set sensors and data
    smodel.sensors.sensor_db.update({
        kpkg.SenKeys.anv : sen_anv,
        kpkg.SenKeys.los : sen_los,
        kpkg.SenKeys.ref : sen_ref
    })

    # set sensors and measurements keys
    smodel.sen_keys = kpkg.sen_keys_list
    smodel.sen_cov_keys = kpkg.sen_cov_keys_list
    smodel.sensors.keymap = kpkg.pro_sen_keymap
