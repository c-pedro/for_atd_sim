''' Database of attitude and attitude rate sensors

    Classes
    -------
    RateGyroDiscrete
        Rate gyro with bias sensor model.

'''
import copy
import numpy as np
from .sensor import SensorModel


class RateGyroDiscrete(SensorModel):
    ''' Rate gyro with bias sensor model.

        Instance Variables
        ------------------
        bias
            Sensor bias or drift.
        sdev_bias
            Sensor bias standard deviation.
        sdev_scale
            Sensor scale standard deviation.

        Public Methods
        --------------
        measure
            Return angular velocity given a ground truth.

        References
        ----------
            1. R.L. Farrenkopf, Analytic Steady-State Accuracy Solutions for Two
            Common Spacecraft Attitude Estimators
            (https://arc.aiaa.org/doi/abs/10.2514/3.55779)

            2. Fundamentals of Spacecraft Attitude Determination and Control
            Markley, Crassidis
            Page 59
            (https://doi.org/10.1007/978-1-4939-0802-8)

    '''
    __DEF_SDEV_ANV = 0
    __DEF_SDEV_BIAS = 0
    __DEF_TIME_STEP = 0.
    __DEF_INIT_BIAS = np.zeros((3,1))


    def __init__(self,
                bias=__DEF_INIT_BIAS,
                sdev_bias=__DEF_SDEV_BIAS,
                sdev_scale=__DEF_SDEV_ANV):
        ''' Set initial properties. By default everything is zero.'''
        super().__init__()

        self.bias = bias
        self.sdev_bias = sdev_bias
        self.sdev_scale = sdev_scale
        self.time_step = self.__DEF_TIME_STEP
        

    def measure(self, true_anv):
        ''' Return angular velocity measurement and save new bias.

            Arguments
            ---------
            true_anv
                Angular velocity ground truth.

            Returns
            -------
            meaAnv
                Angular Velocity Measurement
        '''
        # parse time step from kwargs
        dtime = self.time_step

        # parse old bias
        old_bias = copy.deepcopy(self.bias)

        # get new bias
        new_bias = self.get_next_bias(dtime)

        # get measurement
        sen_anv = self.get_next_anv(dtime, true_anv, new_bias, old_bias)

        # save bias
        self.bias = new_bias

        return sen_anv


    def get_next_bias(self, dtime):
        ''' Return the current bias.

            Based on a discrete rate gyro model.

            Arguments
            ---------
            dtime
                Time interval since last measurement.

            Returns
            -------
            newBias
                Current bias.
        '''
        # sample noise (standard gaussian distribution)
        bias_noise = np.random.standard_normal((3,1))

        new_bias = self.bias + self.sdev_bias * np.sqrt(dtime) * bias_noise

        return new_bias


    def get_next_anv(self, dtime, true_anv, new_bias, old_bias):
        ''' Return the angular velocity measurement.

            Based on a discrete rate gyro model.

            Arguments
            ---------
            dtime
                Time interval since last measurement.
            true_anv
                Angular velocity true value.
            new_bias
                Bias at the time of measurement.

            Returns
            -------
            sen_anv
                Angular velocity measurement.
        '''
        # noise sample (standard gaussian distribution)
        anv_noise = np.random.standard_normal((3,1))

        # compute next angular velocity
        sen_anv = (
            true_anv
            + 0.5 * (new_bias + old_bias)
            + np.sqrt(
                self.sdev_scale**2 / dtime
                + self.sdev_bias**2 * dtime / 12
            ) * anv_noise
        )

        return sen_anv
