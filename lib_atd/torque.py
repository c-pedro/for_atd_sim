''' Torque functions'''
import numpy as np
from for_atd_sim.core.keys import get_key
from for_atd_sim.lib_math.random import random_unit_3d_vec

# module keys
KEY_TOR = 'torque'
KEY_TOR_FUN = get_key('torque','function')
KEY_TOR_PAR = get_key('torque','params')
KEY_TOR_PAR_ITIME = get_key('torque','function','initialtime')
KEY_TOR_PAR_DELTA = get_key('torque','function','delta')
KEY_TOR_PAR_GAIN = get_key('torque','function','gain')
KEY_TOR_PAR_FREQ = get_key('torque','function','frequency')
KEY_TOR_PAR_PHASE = get_key('torque','function','phase')
KEY_TOR_PAR_INI = get_key('torque','function','initial')

TOR_PAR_SET = [KEY_TOR_PAR_GAIN, KEY_TOR_PAR_FREQ, KEY_TOR_PAR_PHASE, KEY_TOR_PAR_INI]

# keys for torque models function
KEY_TORFUN_SIN = 'torque_sinusoid'
KEY_TORFUN_STA = 'torque_static'
KEY_TORFUN_IMP = 'torque_impulse'


def torque_sinusoid (time : float, **tor_par) -> np.ndarray:
    ''' Return sinusoidal torque value.

        Parameters
        ----------
        time : float
            Current simulation time.
        tor_i : tuple

        Returns
        -------
        np.ndarray
            Torque array.

    '''
    gain = tor_par.get(KEY_TOR_PAR_GAIN, 0.1)
    freq = tor_par.get(KEY_TOR_PAR_FREQ, 1)
    phase = tor_par.get(KEY_TOR_PAR_PHASE,  0)
    direction = tor_par.get(KEY_TOR_PAR_INI, random_unit_3d_vec())

    # normalize direction
    if np.linalg.norm(direction) > 10**-10:
        direction = direction / np.linalg.norm(direction)

    return (gain * np.sin(time * freq + phase)) * direction


def torque_impulse (time : float, **tor_par) -> np.ndarray:
    ''' Returns impulse torque value.

        Parameters
        ----------
        time : float
            Current simulation time.

        Returns
        -------
        np.ndarray
            Torque array.

    '''
    time_i =  tor_par.get(KEY_TOR_PAR_ITIME, 0)
    delta = tor_par.get(KEY_TOR_PAR_DELTA, 0.5)
    gain = tor_par.get(KEY_TOR_PAR_GAIN, 1)
    direction = tor_par.get(KEY_TOR_PAR_INI, np.array([[0], [0], [1]]))

    # normalize direction
    direction = direction / np.linalg.norm(direction)

    if time >= time_i and time <= time_i + delta:
        return gain * direction
    return np.zeros((3,1))


def torque_static(time : float, **tor_par) -> np.ndarray:
    ''' Return constant torque. '''
    return tor_par.get(KEY_TOR_PAR_INI, np.zeros((3,1)))


tor_fun_db = {
    KEY_TORFUN_STA : torque_static,
    KEY_TORFUN_SIN : torque_sinusoid,
    KEY_TORFUN_IMP : torque_impulse,
}
