''' Specific data structures related to the attitude problem.

    Classes
    -------
    AttitudeData

    AngularVelocityData

    AbsoluteDirectionData

    RelativeDirectionData

    InertialRelativeDirectionData

'''
import numpy as np

from for_atd_sim.lib_data.basic_data import DataPoint
from for_atd_sim.lib_data.basic_data import IndexedData


class AttitudeData(DataPoint):
    ''' Attitude data point structure.

        Overrides
        ---------
        validate_entry
            Raise AssertionError if data not a rotation matrix

        Methods
        -------
        setup_data
            Initialize entire attitude database from  minimal set.
        complete_data
            Complete data from a minimal set.
        validate_data
            Test data determinant and orthogonality.
        get_atd
            Return attitude from index. 

    '''
    def setup_data(self, data_in : IndexedData):
        ''' Initialize entire attitude database from a minimal set.

            Arguments
            ---------
            data_in
                Minimal set of attitudes

        '''

        for index in data_in:
            self.data[index] = np.array(data_in[index])
        
        self.complete_data()


    def complete_data(self):
        ''' Complete data from a minimal set. '''
        # set remaining attitudes
        if all([
            (0,1) in self.data,
            (0,2) in self.data,
            (0,3) in self.data
        ]):
            self.data[(1,2)] = self.data[(0,1)].T @ self.data[(0,2)]
            self.data[(1,3)] = self.data[(0,1)].T @ self.data[(0,3)]
            self.data[(2,3)] = self.data[(0,2)].T @ self.data[(0,3)]
            
        elif all([
            (0,1) in self.data,
            (1,2) in self.data,
            (1,3) in self.data
        ]): 
            self.data[(0,2)] = self.data[(0,1)].T @ self.data[(1,2)]
            self.data[(0,3)] = self.data[(0,1)].T @ self.data[(1,3)]
            self.data[(2,3)] = self.data[(1,2)].T @ self.data[(1,3)]
            
        self.validate_data()
    

    def validate_data(self):
        ''' Test data determinant and orthogonality. '''
        for index in self.data:
            determinant = np.linalg.det(self.data[index])
            np.testing.assert_almost_equal(determinant, 1, err_msg='Attitude matrix must be a unit vector.')
            np.testing.assert_almost_equal(self.data[index] @ self.data[index].T, np.eye(3), err_msg='Attitude matrix must be orthogonal.' )


    def get_atd(self, index):
        ''' Return attitude from index. '''
        if index in self.data:
            return self.data[index]
        elif (index[1],index[0]) in self.data:
            return self.data[(index[1],index[0])].T
        raise IndexError



# TODO : add validate method
class AngularVelocityData(DataPoint):
    ''' Angular velocity data point structure.

        Methods
        -------
        setup_data
            Initialize entire attitude database from minimal set.

    '''
    def setup_data(self, idata : IndexedData):
        '''Set or reset entire attitude data.'''
        for index in idata:
            self.data[index] = np.array(idata[index])



class AbsoluteDirectionData(DataPoint):
    ''' Absolute direction data point structure.    

        Overrides
        ---------
        validate_entry
            Raise AssertionError if data not a unit vector

        Methods
        -------
        setup_data
            Set or reset data in instance.

    '''

    def setup_data(self, idata : IndexedData, iatd : AttitudeData):
        ''' Set vectors in body fixed frame from the inertial vectors.

            Arguments
            ---------
            idata
                Inertial absolute direction data array.
            iatd
                Current attitude data point.

        '''
        data_body = {}
        self.data = get_normalized_vectors(idata)

        # compute body values
        for index in self.data:
            # define different indexes
            body_index = (index[0], index[0])
            atd_index = (index[0], 0)

            try: 
                data_body[body_index] = iatd.get_atd(atd_index) @ self.data[index]
            
            except IndexError:
                raise IndexError('Attitude dataset is incomplete, attitude required does not exist.')

        self.data.update(data_body)
        self.validate_data()


    # TODO : repeated in different classes use common
    def validate_data(self):
        '''Test data norm.'''
        for index in self.data:
            norm = np.linalg.norm(self.data[index])
            np.testing.assert_almost_equal(norm, 1, err_msg='Absolute direction must be a unit vector.')



class RelativeDirectionData(DataPoint):
    ''' Relative direction data in body frame.

        Overrides
        ---------
        validate_entry
            Raise AssertionError if data not a unit vector

        Methods
        -------
        setup_data
            Set or reset data in instance.

    '''
    def setup_data(self, inertial_data : IndexedData, iatd : AttitudeData):
        ''' Set data from inertial set of vectors.

            Arguments
            ---------
            inertial_data
                Inertial relative direction data array.
            iatd
                Current attitude data array.

        '''
        inertial_data = get_normalized_vectors(inertial_data)

        for index in inertial_data:
            self.data[index] = iatd.get_atd((index[1], 0)) @ inertial_data[index]
            
            index_t = (index[1], index[0])
            self.data[index_t] = - iatd.get_atd((index[1], 0)) @ inertial_data[index]

        # validate data input
        self.validate_data()


    def validate_data(self):
        '''Test data norm.'''
        for index in self.data:
            norm = np.linalg.norm(self.data[index])
            np.testing.assert_almost_equal(norm, 1, err_msg='Relative direction must be a unit vector.')



class InertialRelativeDirectionData(DataPoint):
    ''' Inertial Relative (between vehicles) direction data point structure.

        Overrides
        ---------
        validate_entry
            Raise AssertionError if data not a unit vector

        Methods
        -------
        setup_data
            Set or reset data in instance.

    '''
    def setup_data(self, inertial_data : IndexedData):
        ''' Set or reset data in instance.

            Arguments
            ---------
            inertial_data
                Inertial relative direction data array.
            iatd
                Current attitude data array.

        '''
        self.data = get_normalized_vectors(inertial_data)

        self.data.update({(index[1], index[0]):-inertial_data[index] for index in inertial_data})

        self.validate_data()


    def validate_data(self):
        '''Test data norm.'''
        for data in self.data:
            norm = np.linalg.norm(self.data[data])
            np.testing.assert_almost_equal(norm, 1, err_msg='Relative direction must be a unit vector.')



# TODO : move appropriate to basic data
def get_normalized_vectors(data : IndexedData):
    ''' Returns data as unit vectors. '''
    for index in data:
        data[index] = np.array(data[index]) / np.linalg.norm(np.array(data[index]))  

    return data
