''' Database of sensors that measure a direction vector.

    Classes
    -------
    FocalPlane
        Wide FOV Focal Plane Detector sensor model.

'''
import numpy as np
from .sensor import SensorModel


class FocalPlane(SensorModel):
    ''' Wide FOV Focal Plane Detector sensor model.

        Instance Variables
        ------------------
        sdev
            Standard deviation of the model.
        sensorToBody
            Orientation of body relative to sensor.
        bodyToSensor
            Orientation of sensor relative to body.

        Public Methods
        --------------
        update_rotation
            Update sensor attitude relative to body.
        measure
            Return current measurement in the body frame.
        frame_sensor_to_focal
            Return the focal coordinates of a sensor frame vector.
        compute_sensor_covariance
            Return the covariance of a sensor frame vector.
        compute_focal_covariance
            Returns the focal covariance for wide FOV.
        compute_jacobian
            Returns the Jacobian relative to the sensor frame.
        sample_sensor_noise
            Sample sensor noise from a normal distribution.
        get_sensor_from_cube
            Return the body to sensor frame rotation.

        References
        ----------
            1. [1] Attitude Estimation for Large Field-of-View Sensors
            Cheng, Crassidis, Markley
            2006
            (https://doi.org/10.1007/BF03256499)
            2. Kalman Filtering of Spacecraft Attitude and the QUEST Model
            Shuster
            1990
            (http://malcolmdshuster.com/Pub_1990d_J_kfq_scan.pdf)

    '''
    # default boresight axis (from model)
    __DEF_BORESIGHT = np.array([[0],[0],[1]])
    # default noise mean
    __DEF_NOISE_MEAN_FOCAL = np.zeros((2,1))
    __DEF_NOISE_MEAN_SENSOR = np.zeros((3,1))

    KEY_CUBE = 'cube'

    def __init__(self, sdev = 0, bodyToSensor = np.eye(3), **kargs):
        ''' If given save standard deviation, sensor orientation and options.

            Arguments
            ---------
            sdev (optional)
                Standard deviation
            bodyToSensor (optional)
                Body to sensor rotation matrix.
            **kargs (optional)
                Options.

        '''
        super().__init__()
        # model standard deviation
        self.sdev = sdev
        # sensor orientation (useful for gimbaled)
        self.bodyToSensor = bodyToSensor
        self.sensorToBody = bodyToSensor.T
        # options
        self.sensorFrame = self.KEY_CUBE


    def update_rotation(self, bodyToSensor):
        ''' Update sensor attitude relative to body.'''
        self.bodyToSensor = bodyToSensor
        self.sensorToBody = bodyToSensor.T


    def measure(self, tBodyVec, **kargs):
        ''' Return current measurement in the body frame.

            Arguments
            ---------
            tBodyVec
                True vector in body frame.
            arg 2 (optional)
                [one line summary]
            **kargs
                [one line summary and source]

            Returns
            -------
            mBodyVec
                Measurement vector in body frame.

        '''
        # select a good sensor orientation
        if self.sensorFrame == self.KEY_CUBE:
            self.bodyToSensor = self.get_sensor_from_cube(tBodyVec)
            self.sensorToBody = self.bodyToSensor.T

        # body frame to sensor frame
        tSensorVec = self.bodyToSensor @ tBodyVec
        # parse current mean
        sen_mean = self.__DEF_NOISE_MEAN_SENSOR
        # compute current covariance
        sen_cov = self.compute_sensor_covariance(tSensorVec)
        # sample sensor noise
        sen_noise = self.sample_sensor_noise(sen_mean, sen_cov)
        # compute sensor measurement
        mea_svec = tSensorVec + sen_noise
        # compute body measurement
        mea_bvec = self.sensorToBody @ mea_svec

        # rotate covariance
        sen_cov = self.sensorToBody @ sen_cov @ self.sensorToBody.T

        # get unit vector?
        mea_bvec = mea_bvec / np.linalg.norm(mea_bvec)

        # save covariance
        self.cov = sen_cov

        return mea_bvec


    def frame_focal_to_sensor(self, focalVec):

        # get focal coordinates
        alpha = focalVec[0,0]
        beta = focalVec[1,0]
        sensorVec = 1 / np.sqrt(1+alpha**2+beta**2) * np.array([[-alpha],[-beta],[1]])
        return sensorVec


    def frame_sensor_to_focal(self, sensorVec):
        ''' Return the focal coordinates of a sensor frame vector.

            Arguments
            ---------
            sensorVec
                Unit vector in sensor frame.

            Returns
            -------
            focalVec
                Focal coordinates vector.

            Raises
            ------
            AssertionError
                Direction not in the field of view of sensor (z<0).

        '''
        # get norm of focal coordinates
        norm = sensorVec[2,0]

        # check if measurement is in sensor range
        if norm <= 0:
            raise AssertionError

        # get focal coordinates
        alpha = - sensorVec[0,0] / norm
        beta = - sensorVec[1,0] / norm

        focalVec = np.array([[alpha], [beta]])
        return focalVec


    def compute_sensor_covariance(self, sensorVec):
        ''' Return the covariance of a sensor frame vector.

            Arguments
            ---------
            sensorVec
                Unit vector in sensor frame.

            Returns
            -------
            sensorCov
                Covariance of the measurement in the sensor frame.

        '''
        # get focal coordinates
        focalVector = self.frame_sensor_to_focal(sensorVec)
        alpha = focalVector[0,0]
        beta = focalVector[1,0]
        # get Jacobian
        jacobian = self.compute_jacobian(sensorVec, alpha, beta)
        # compute focal covariance
        focalCov = self.compute_focal_covariance(alpha, beta)
        # compute sensor covariance
        sensorCov = jacobian @ focalCov @ jacobian.T

        return sensorCov


    def compute_focal_covariance(self, alpha, beta):
        ''' Returns the focal covariance for wide FOV.'''
        sdev = self.sdev
        scale = sdev**2 * (1 + alpha**2 + beta**2)
        matrix = np.array([[1+alpha**2, alpha*beta],
                           [alpha*beta, 1+beta**2]])
        return scale * matrix


    def compute_jacobian(self, vector, alpha, beta):
        ''' Returns the Jacobian relative to the sensor frame.'''
        scale1 = 1 / np.sqrt(1 + alpha**2 + beta**2)
        scale2 = - 1 / (1 + alpha**2 + beta**2)

        matrix1 = np.array([[-1,  0],
                            [ 0, -1],
                            [ 0,  0]])

        matrix2 = vector @ np.array([[alpha, beta]])

        return scale1 * matrix1 + scale2 * matrix2


    def sample_sensor_noise(self, mean, covariance):
        ''' Sample sensor noise from a normal distribution.

            Arguments
            ---------
            mean
                Noise mean value.
            covariance
                Noise covariance.

            Returns
            -------
            noise2D
                Sampled noise as a 2D column vector.

            Follows guidance from:
            (https://numpy.org/doc/stable/reference/random/index.html)

        '''
        rng = np.random.default_rng()
        noise1D = rng.multivariate_normal(mean[:,0], covariance)
        noise2D = np.array([noise1D]).T
        return noise2D


    def get_sensor_from_cube(self, bodyVec):
        ''' Return the body to sensor frame rotation.

            Arguments
            ---------
            bodyVec
                Unit vector in the body frame.

            Returns
            -------
            bodyToSensor
                Body to sensor frame rotation.

        '''
        # get index of biggest value
        ind = np.unravel_index(
            np.argmax(np.abs(bodyVec), axis=None),
            bodyVec.shape
        )
        sgn = np.sign(bodyVec[ind])

        if ind[0] == 0:
            if sgn == 1:
                bodyToSensor = np.array([[0, 0, -1],
                                         [0, 1,  0],
                                         [1, 0,  0]])
            else:
                bodyToSensor = np.array([[ 0, 0, 1],
                                         [ 0, 1, 0],
                                         [-1, 0, 0]])
        elif ind[0] == 1:
            if sgn == 1:
                bodyToSensor = np.array([[1, 0,  0],
                                         [0, 0, -1],
                                         [0, 1,  0]])
            else:
                bodyToSensor = np.array([[1,  0, 0],
                                         [0,  0, 1],
                                         [0, -1, 0]])
        elif ind[0] == 2:
            if sgn == 1:
                bodyToSensor = np.array([[1, 0, 0],
                                         [0, 1, 0],
                                         [0, 0, 1]])
            else:
                bodyToSensor = np.array([[-1, 0,  0],
                                         [ 0, 1,  0],
                                         [ 0, 0, -1]])

        return bodyToSensor
