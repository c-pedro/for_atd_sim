import io
import re
from pathlib import Path
from copy import deepcopy
from typing import Tuple, List

import numpy as np

from for_atd_sim.lib_data.basic_data import DataSeries, DataPoint
from for_atd_sim.lib_data.basic_data import IndexedData
from for_atd_sim.core.keys import FLD_SEP


class FileCsv():
    ''' Interface to extract data from a csv file.

    '''
    CSV_FLD_KEYS = 'keys'
    CSV_FLD_TIME = 'time'
    CSV_FLD_INDX = 'index'
    CSV_FLD_VALS = 'values'

    def __init__(self):
        self.csv_sep = ';'

    @staticmethod
    def get_data_series_key(path_file: Path):
        return path_file.stem.split(FLD_SEP)[-1]


    def get_data_from_file(self, path_file: Path) -> dict[str, IndexedData]:
        ''' Returns data in file. '''

        #  read lines and parse data
        with open(Path(path_file), mode='r') as mfile:

            # check header
            header = mfile.readline().replace(' ', '').rstrip('\n')

            # get fields
            header = header.split(self.csv_sep)

            if self.is_series_header(header):
                mfile.seek(0, io.SEEK_SET)
                return self.get_data_series(mfile)

            # dict TODO test is dict
            if self.is_dict_header(header):
                return self.get_data_dict(mfile)
            
            if self.is_indexed_dict_header(header):
                mfile.seek(0, io.SEEK_SET)
                return self.get_indexed_data_dict(mfile)

            if self.is_headless_dict(header):
                mfile.seek(0, io.SEEK_SET)
                return self.get_data_dict(mfile)

            raise TypeError


    def get_data(self, mfile: io.FileIO) -> object:
        ''' Return data in appropiate format.

            dict or DataSeries

            Parameters
            ----------
            mfile : io.FileIO
                Data file object.

            Returns
            -------
            object
                Data.
        '''
        # check header
        header = mfile.readline().replace(' ', '').rstrip('\n')

        # define separator
        for isep in [',', ';', '|']:
            if isep in header:
                self.csv_sep = isep
                break

        # get fields
        header = header.split(self.csv_sep)

        # dict TODO test is dict
        if self.is_dict_header(header):
            return self.get_data_dict(mfile)

        if self.is_entrylist_header(header):
            return self.get_entry_list_data(mfile)

        if self.is_series_header(header):
            return self.get_data_series(mfile)

        raise TypeError


    def write_dict(self, path_file: Path, data: dict):
        with open(Path(path_file), mode='w') as mfile:
            for key,value in data.items():
                self.write_row(mfile, [key, str(value)])


    def write_dict_index(self, path_file: Path, data: dict[str,IndexedData]):
        with open(Path(path_file), mode='w') as mfile:
            for key,value in data.items():
                for index, idxval in value.items():
                    
                    if isinstance(idxval, np.ndarray):
                        str_idxval = np.array2string(idxval, separator=',').replace('\n', '')
                    elif isinstance(idxval, list):
                        str_idxval = self.get_list_str(idxval)
                        
                        # str_idxval = ''
                        # for listval in idxval:
                        #     if isinstance(listval, np.ndarray):
                        #         str_idxval += np.array2string(listval, separator=',').replace('\n', '')
                                
                        #     else:
                        #         str_idxval += str(listval).replace('\n', '')
                    
                    
                    else:
                        str_idxval = str(idxval).replace('\n', '')
                    
                    str_value = (
                        '{' 
                        + str(index) 
                        + ':'
                        + str_idxval
                        + '}'
                    )
                    
                    self.write_row(mfile, [key, str_value])
            

    def get_list_str(self, ilist : list):
        str_idxval = '['
        
        for ival in ilist:
            if isinstance(ival, list):
                str_idxval += self.get_list_str(ival) + ','
            elif isinstance(ival, np.ndarray):
                str_idxval += np.array2string(ival, separator=',').replace('\n', '')
            else:
                str_idxval += str(ival).replace('\n', '') + ','
        
        return str_idxval.rstrip(',') + ']'


    def write_data_series(self, path_file: Path, data_series: DataSeries):
        with open(Path(path_file), mode='w') as mfile:
            # write header
            index_list = [index for index in data_series.series[0].data]
            header = ['time'] + [str(index) for index in index_list]
            self.write_row(mfile, header)

            # write data
            for dpnt in data_series.series:
                data_list = (
                    [np.array2string(np.float64(dpnt.time), precision=4)]
                    + [
                        np.array2string(dpnt.data[index], separator=',').replace('\n', '')
                        for index in index_list
                    ]
                )
                self.write_row(mfile, data_list)


    def write_row(self, mfile: io.FileIO, row_list: List[str]):
        for val in row_list[:-1]:
            mfile.write(str(val) + self.csv_sep)
        mfile.write(row_list[-1] + '\n')


    def read_data_series(self, path_file: Path) -> DataSeries:
        with open(Path(path_file), mode='r') as mfile:
            return self.get_data_series(mfile)


    def get_data_series(self, mfile: io.FileIO) -> DataSeries:
        ''' Returns DataSeries from file.

            Assume first column is time

            Parameters
            ----------
            mfile : file object
                File object with the data.

            Returns
            -------
            DataSeries
                Data series with all data of the file.

            TODO : problems with nan?

        '''
        file_data = DataSeries()

        # parse header
        header = mfile.readline()  # .replace(' ','')
        index_list = self.get_index_list(header)

        # parse data points
        while True:
            row = mfile.readline()

            if row == '':
                return file_data

            data_list = self.get_row_data(row)

            row_dpnt = DataPoint()
            row_dpnt.time = data_list.pop(0)
            row_dpnt.data = {index_list[k]: np.array(
                data_list[k]) for k in range(len(data_list))}

            file_data.series.append(deepcopy(row_dpnt))


    def get_index_list(self, line: str) -> List[Tuple[int, int]]:
        ''' Returns list of indices. '''
        re_indices = re.findall(r'[(](\d*),\s*(\d*)[)]', line)

        if len(re_indices) == 0:
            raise ValueError
        else:
            return [(int(index[0]), int(index[1])) for index in re_indices]


    def get_row_data(self, row: str) -> List[Tuple[int, int]]:
        ''' Returns list data. '''
        re_data = re.findall('([^{}]+)'.format(self.csv_sep), row)

        return [np.safe_eval(data_val) for data_val in re_data]


    def get_data_dict(self, mfile: io.FileIO) -> dict:
        ''' Return data in csv file with dict. '''
        # initialize dict
        self.data_input = {}

        # TODO : define tipe of data (from key)

        # read first line
        iline = mfile.readline()

        while iline != '':
            # get key and value
            idata = iline.rstrip('\n').split(self.csv_sep)

            # add to dict
            self.data_input[idata[0]] = np.safe_eval(idata[1])

            # read next line
            iline = mfile.readline()

        return self.data_input


    def get_indexed_data_dict(self, mfile: io.FileIO) -> dict:
        ''' Return data in csv file with dict. '''
        # initialize dict
        self.data_input = {}

        # TODO : define tipe of data (from key)

        # read first line
        iline = mfile.readline()

        while iline != '':
            # get key and value
            ikey, idict = iline.rstrip('\n').split(self.csv_sep)
            idx, idata = idict.lstrip('{').rstrip('}').split(':')

            # add to dict
            if ikey not in self.data_input:
                self.data_input[ikey] = {}

            try:
                self.data_input[ikey][np.safe_eval(idx)] = np.safe_eval(idata)
            except ValueError:
                self.data_input[ikey][np.safe_eval(idx)] = str(idata)

            # read next line
            iline = mfile.readline()

        return self.data_input


    def is_dict_header(self, fld_list) -> bool:
        if len(fld_list) != 2:
            return False
        if fld_list[0] != self.CSV_FLD_KEYS:
            return False
        if fld_list[1] != self.CSV_FLD_VALS:
            return False
        return True
    

    def is_indexed_dict_header(self, fld_list) -> bool:
        if len(fld_list) != 2:
            return False
        if fld_list[1][0] != '{':
            return False
        if fld_list[1].rstrip('\n')[-1] != '}':
            return False
        return True


    def is_headless_dict(self, fld_list) -> bool:
        if len(fld_list) != 2:
            return False
        return True


    def is_series_header(self, fld_list) -> bool:
        if fld_list[0] != self.CSV_FLD_TIME:
            return False
        return True
