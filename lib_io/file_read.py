''' This module contains functions for reading data from a file.
    
'''
from typing import Union
from pathlib import Path
from for_atd_sim.lib_data.basic_data import IndexedData, DataSeries
from for_atd_sim.lib_io.file_data import FileData
from for_atd_sim.lib_io.file_csv import FileCsv


def read_file(path_file : Path) -> Union[dict[str, IndexedData], DataSeries]:
    ''' Returns a dictionary with the data in the file.

    '''
    if path_file.suffix == '.md':
        file_reader = FileData()
    elif path_file.suffix == '.csv':
        file_reader = FileCsv()

    return file_reader.get_data_from_file(path_file)


def read_data_series(path_file : Path) -> DataSeries:
    file_reader = FileCsv()
    return file_reader.read_data_series(path_file)
   