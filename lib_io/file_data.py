import re
import warnings
from io import FileIO
from pathlib import Path
from typing import Tuple, Union

import numpy

from for_atd_sim.lib_data.basic_data import IndexedData
from for_atd_sim.lib_math.so3 import rot_mtx

class DataEntry:
    key = ''
    index = None
    data : IndexedData = {}


class FileData():
    ''' This class interfaces the input file with the application.

        ## Variables
        * name  [string]
        * index [array]
        * value [scalar or array]
    
    '''
    VALID_ENTRY_INI_CHAR = '```'
    VALID_ENTRY_END_CHAR = '```'
    VALID_LANGUAGE_LIST = ['object', 'python']

            
    def get_data_from_file(self, path_file: Path) -> dict[str, IndexedData]:
        ''' Returns all data in file. '''
        data_file:  dict[str, IndexedData] = {}

        #  read lines and parse data
        with open(Path(path_file), mode='r') as mfile:
            while True:
                try:
                    ientry = self.get_next_entry(mfile)
                except EOFError:
                    break

                # save entry in database
                if ientry.key not in data_file:
                    data_file[ientry.key] = {}
                data_file[ientry.key].update({ientry.index: ientry.data})

        return data_file


    def get_next_entry(self, mfile: FileIO) -> DataEntry:
        ''' Returns the next valid data entry. '''
        
        data_entry = DataEntry()

        data_format = self.find_next_entry(mfile)
        # object format as default
        if data_format is None:
            data_format = self.VALID_LANGUAGE_LIST[0]

        # read name and index
        data_entry.key, data_entry.index = self.get_name_index(
            mfile.readline())

        # select method to read data
        if data_format == self.VALID_LANGUAGE_LIST[0]:
            data_entry.data = self.get_object_data(mfile)
        elif data_format == self.VALID_LANGUAGE_LIST[1]:
            data_entry.data = self.get_python_data(mfile)

        # TODO put methods in the function that extracts respective fields
        # data conformity validation
        if data_entry.key == '':
            warnings.warn('Reading no name. Returned empty entry.')
            data_entry = DataEntry()
        elif data_entry.data is None:
            warnings.warn('Reading no data. Returned empty entry.')
            data_entry = DataEntry()
        elif re.match(r'[(][0-9,]*[)]', data_entry.key) is not None:
            warnings.warn(
                'Index format not accepted as name. Returned empty entry.')
            data_entry = DataEntry()
        elif data_entry.index is not None:
            if re.match(r'([(]\d*,\s*\d[)])', str(data_entry.index)) is None:
                warnings.warn('Invalid index format. Returned empty entry.')
                data_entry = DataEntry()

        return data_entry


    def find_next_entry(self, mfile : FileIO) -> Union[None,str]:
        ''' Finds start of entry and returns format if specified. '''
        # find next entry
        while True:
            line = mfile.readline()

            # end of file
            if line == '':
                raise EOFError

            # found entry
            elif FileData.VALID_ENTRY_INI_CHAR in line:

                # read data format
                for entry_language in self.VALID_LANGUAGE_LIST:
                    if entry_language in line:
                        return entry_language

                return


    def get_name_index(self, line) -> Tuple[str, str]:
        ''' Returns name and index from string. '''
        name: str = self.get_name(line)
        index: Tuple[int, int] = self.get_index(line)

        return name, index


    def get_name(self, line: str) -> str:
        ''' Returns key of data entry. '''
        return re.findall(r'(\w*)[\W]+', line)[0]


    def get_index(self, line) -> Union[Tuple[int, int], None]:
        ''' Returns index if exists. '''
        re_indexes = re.findall(r'[(](\d*),(\d*)[)]', line)

        if len(re_indexes) == 0:
            return None
        else:
            return (int(re_indexes[0][0]), int(re_indexes[0][1]))


    def get_object_data(self, mfile: FileIO) -> numpy.ndarray:
        ''' Returns array of data from object entry. '''
        my_data = []

        while True:
            line = mfile.readline()

            if FileData.VALID_ENTRY_END_CHAR in line:
                break

            my_data.append(self.get_value_line(line))

        return numpy.array(my_data)


    def get_value_line(self, line: str) -> list:
        ''' Return list of floats from line. '''
        # assume whitespaces separate values
        val_arr = re.findall(r'(\d+\.*\d*)\s*', line)

        return [float(val) for val in val_arr]


    def get_python_data(self, mfile: FileIO) -> numpy.ndarray:
        ''' Returns data in from python expression.'''
        line = ''

        while self.VALID_ENTRY_END_CHAR not in line:
            line += mfile.readline()
            # line = line.rstrip('\n')

        line = line.replace(self.VALID_ENTRY_END_CHAR, '')

        # TODO : ensure no operations in input file
        #   return numpy.safe_eval(line)
        return eval(line)
