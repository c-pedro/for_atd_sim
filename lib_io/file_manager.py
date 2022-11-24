''' Manage input and results files.
'''
import os
import errno
import shutil
from pathlib import Path
from typing import List, Tuple

import for_atd_sim.core.keys as km
from for_atd_sim.lib_io.file_csv import FileCsv
from for_atd_sim.lib_io.file_read import read_file
from for_atd_sim.core.simid import SimID
from for_atd_sim.lib_atd import trf_orthogonalize_rotation_list
from for_atd_sim.lib_data import DataSeries

ROT_KEY_LIST = [
    km.ProKeys.atd,
    km.SenKeys.atd,
    km.ObsKeys.atd,
    km.ObsKeys.atd_sen,
    km.ObsKeys.atd_err,
]


class FileManager:
    ''' Manage naming and directory structure of IO files. '''

    KEY_TAG_SIM = 'sim'
    KEY_TAG_ANL = 'anl'
    KEY_TAG_DICT = 'dict'
    KEY_TAG_STATS = 'stats'

    DIR_FIGURE = 'figs'

    def __init__(self) -> None:
        self.name_man = self.NamingManager()

    def get_file_list(self, output_path: Path, serid: int) -> List[Path]:
        ''' Return list of the path of all files for given series. '''

        # initialize file list
        filelist_output = []

        # select series directory
        series_path = self.series_path(output_path, serid)

        # append files to list
        str_list = os.listdir(series_path)

        for ifilename in str_list:
            # accept only if name is valid
            if self.name_man.is_filepath_valid(Path(series_path, ifilename)):
                filelist_output.append(Path(series_path, ifilename))

        return filelist_output

    def find_files_by_id(self, output_path: Path, simid: SimID) -> List[Path]:
        ''' Return list of files with a specific ID.'''

        id_file_list = []

        series_files = self.get_file_list(output_path, simid.ser)

        for ifile in series_files:
            [file_id, _, _] = self.name_man.get_metadata(ifile.stem)

            file_id.ser = simid.ser

            if file_id == simid:
                id_file_list.append(ifile)

        return id_file_list

    def find_files_same_gid(self, output_path: Path, simid: SimID) -> List[Path]:
        ''' Return list of files with the same  TrueID.'''
        id_file_list = []

        series_files = self.get_file_list(output_path, simid.ser)

        for ifile in series_files:
            [file_id, _, _] = self.name_man.get_metadata(ifile.stem)

            file_id.ser = simid.ser

            if file_id.tru == simid.tru:
                id_file_list.append(ifile)

        return id_file_list

    def find_file(self, output_path: Path, simid: SimID, key_search: str) -> Path:
        ''' Find a file using id and key. '''

        files_id = self.find_files_by_id(output_path, simid)

        for ifile in files_id:
            [_, _, ikey] = self.name_man.get_metadata(ifile.stem)

            if ikey == key_search:
                return ifile

        raise FileNotFoundError(
            errno.ENOENT, os.strerror(errno.ENOENT), key_search
        )

    def find_import_series(self, output_path: Path, simid: SimID) -> List[Path]:
        '''Returns list of Paths to files to import by id. '''

        same_id_files = self.get_file_list(output_path, simid.ser)

        # remove anl and obs
        return [ifile for ifile in same_id_files if self.is_import(ifile.stem)]

    def import_files(self, file_list: List[Path], output_path: Path, simid: SimID):
        for ifile in file_list:
            if ifile.is_file():
                shutil.copy(ifile, self.series_path(output_path, simid.ser))

    def series_path(self, output_path, serid: int):
        ''' Return the path of the series output. '''
        return Path(output_path, self.name_man.get_serid_str(serid))

    def figs_path(self, output_path, serid):
        ''' Return the path of the series figures output. '''
        figs_path = Path(self.series_path(output_path, serid), self.DIR_FIGURE)
        if not figs_path.exists():
            figs_path.mkdir()
        return figs_path

    def get_id_list(self, output_path: Path, serid: int):
        ''' Retrun list with all ids in path. '''
        simid_list: List[SimID] = []

        ser_flist = self.get_file_list(output_path, serid)

        for ifile in ser_flist:
            [id_ifile, _, _] = self.name_man.get_metadata(ifile.stem)

            if id_ifile not in simid_list:
                simid_list.append(id_ifile)

        return simid_list

    def clear_series_data(self, output_path: Path, serid: int) -> None:
        ''' Remove all files in the output of a series and its subdirectories. '''
        remove_all_subfiles(self.series_path(output_path, serid))

    def clear_prior_trueid(self, output_path: Path, serid: int):
        ''' Delete data associated with the prior true values. '''
        last_id = self.find_last_grnid(output_path, serid)

        list_file_path = self.find_files_same_gid(output_path, last_id)

        for ifile in list_file_path:
            os.remove(ifile.absolute())

    def orthogonalize(self, data_ser : DataSeries) -> DataSeries:
        ''' Orthogonalize rotation matrices series. '''
        if not isinstance(data_ser, DataSeries):
            return data_ser

        otho_ser = trf_orthogonalize_rotation_list(data_ser)
        return otho_ser

    def load_file_data(self, file_path):
        ''' Load specific input file. '''
        return read_file(file_path)

    def load_single_file(self, output_path: Path, simid: SimID, key_search: str) -> dict:
        ''' Returns dict with data from file. 

            Requires that the id is exactly as the file. (-1 if id not important)

        '''
        file_path = self.find_file(output_path, simid, key_search)

        data = self.load_file_data(file_path)
        
        if key_search in ROT_KEY_LIST:
            data = self.orthogonalize(data)

        return data

    def save_csv_files(self, output_path: Path, file_id: SimID, db_data: dict, file_tag=KEY_TAG_SIM):
        ''' Save data series in csv file. '''

        # TODO : perhaps necessary
        # if self.KEY_FNAME_EXTRA in kwargs:
        #     extra = km.CHAR_SEP + kwargs[self.KEY_FNAME_EXTRA]
        # else:
        #     extra = ''

        # TODO : what is this?
        # write_kwargs = {}
        # if self.OPT_ATTRIBUTE_WRITE in kwargs:
        #     write_kwargs.update({
        #         self.OPT_ATTRIBUTE_WRITE : kwargs[self.OPT_ATTRIBUTE_WRITE]
        #     })

        # TODO method in naming
        # generate filename
        for file_key in db_data:

            # FIXME: no s00001 if other exist
            file_name = self.name_man.get_filename(
                file_id, file_tag, file_key, extra='')

            # get full path
            file_path = Path(self.series_path(
                output_path, file_id.ser), file_name+'.csv')

            # write file
            # TODO : distinguish between csv and dict files
            # TODO : simplify write

            # create directory
            if not file_path.parent.exists():
                file_path.parent.mkdir()

            csv_man = FileCsv()
            csv_man.write_data_series(
                file_path,
                db_data[file_key]
            )

    def save_csv_dict(self, output_path: Path, file_id: SimID, db_data: dict, file_key, file_tag=KEY_TAG_SIM):
        ''' Save dict in a csv file. '''

        file_name = self.name_man.get_filename(
            file_id, file_tag, file_key, extra='')

        # get full path
        file_path = Path(self.series_path(
            output_path, file_id.ser), file_name+'.csv')

        # create directory
        if not file_path.parent.exists():
            file_path.parent.mkdir()

        csv_man = FileCsv()
        csv_man.write_dict(
            file_path,
            db_data
        )

    def save_csv_params(self, output_path: Path, file_id: SimID, db_data: dict, file_key, file_tag=KEY_TAG_SIM):
        ''' Save dict in a csv file. '''

        file_name = self.name_man.get_filename(
            file_id, file_tag, file_key, extra='')

        # get full path
        file_path = Path(self.series_path(
            output_path, file_id.ser), file_name+'.csv')

        # create directory
        if not file_path.parent.exists():
            file_path.parent.mkdir()

        csv_man = FileCsv()
        csv_man.write_dict_index(
            file_path,
            db_data
        )

    def find_last_grnid(self, output_path: Path, serid: int) -> SimID:
        ''' Returns the id of the last data available for the groundtruth'''
        last_id = SimID(serid)

        # list of all ids in path
        id_list = self.get_id_list(output_path, serid)

        for id_item in id_list:
            # ignore data unrelated to groundtruth
            if id_item.tru < 0:
                continue

            # save highest tru id
            if id_item.tru > last_id.tru:
                last_id.tru = id_item.tru

        return last_id

    def find_last_meaid(self, output_path: Path, serid: int) -> SimID:
        ''' Returns the id of the last available measurement set.'''
        last_id = SimID(serid)

        # there is no measurement without true values
        last_id = self.find_last_grnid(output_path, serid)

        # list of all ids in path
        id_list = self.get_id_list(output_path, serid)

        for id_item in id_list:
            # ignore data unrelated to groundtruth
            if id_item.mea < 0 or id_item.tru < 0:
                continue

            # save highest tru id
            if id_item.mea > last_id.mea:
                last_id.mea = id_item.mea

        return last_id

    def next_gid(self, output_path: Path, simid: SimID) -> SimID:
        ''' Returns the next available true values id in the series. '''
        next_id = SimID(simid.ser)

        # get last id
        last_id = self.find_last_grnid(output_path, simid.ser)

        # list of all ids in path
        id_list = self.get_id_list(output_path, simid.ser)

        for id_item in id_list:
            # ignore data unrelated to groundtruth
            if id_item.tru > simid.tru and id_item.tru <= last_id.tru:
                next_id.tru = id_item.tru
                last_id.tru = id_item.tru
                next_id.mea = simid.mea # same as input

        return next_id

    def is_import(self, fname: str) -> bool:
        fid, ftag, fkey = self.name_man.get_metadata(fname)

        if ftag not in [self.KEY_TAG_SIM, self.KEY_TAG_DICT]:
            return False
        if km.ObsKeys.prefix in fkey:
            return False

        return True

    class NamingManager:
        ''' Manage naming of each file. 

            Convention
                id-tag-key.extension

            id : unique identifier of simulation 
            tag : origin of data
            key : data reference

        '''

        # prefixes
        pre_sid = 's'
        pre_gid = 'g'
        pre_mid = 'm'

        # seprators
        FIELD_SEPARATOR = km.FLD_SEP
        COMPONENT_SEPARATOR = km.CHAR_SEP

        def get_grnid_str(self, gid: int) -> str:
            return self.gen_id_str(self.pre_gid, gid)

        def get_meaid_str(self, mid: int) -> str:
            return self.gen_id_str(self.pre_mid, mid)

        def get_serid_str(self, sid: int) -> str:
            return self.gen_id_str(self.pre_sid, sid)

        @staticmethod
        def gen_id_str(prefix: str, id_int: int, nlen=5) -> str:
            ''' Returns the ID string for file names.

                ID string length is constant. Zeros are added to the left.

                Parameters
                ----------
                prefix : str
                    ID prefix.
                id_int : int
                    ID number.
                nlen : int, optional
                    ID length, by default 5

                Returns
                -------
                str
                    Formatted ID string.

                Raises
                ------
                AssertionError 
                    if length os string not correct

            '''
            id_str = str(id_int)

            while len(id_str) < nlen:
                id_str = '0' + id_str

            assert len(id_str) == nlen

            return prefix + id_str

        def get_file_id_str(self, file_id: SimID):
            ''' Returns string of file id. '''

            id_valid = []
            id_str = ''

            if file_id.ser > -1:
                id_valid.append(self.gen_id_str(self.pre_sid, file_id.ser))
            if file_id.tru > -1:
                id_valid.append(self.gen_id_str(self.pre_gid, file_id.tru))
            if file_id.mea > -1:
                id_valid.append(self.gen_id_str(self.pre_mid, file_id.mea))

            for id_fld in id_valid:
                id_str += self.COMPONENT_SEPARATOR + id_fld

            return id_str.lstrip(self.COMPONENT_SEPARATOR)

        def get_metadata(self, filename: str) -> Tuple[SimID, str, str]:
            ''' Returns id, tag, and key from filename. 

                filename must not have line end nor file extension. 

                Returns
                -------
                file_id, tag, key
            '''
            [id_str, tag, key] = filename.split(self.FIELD_SEPARATOR)

            file_id = self.get_id_from_str(id_str)

            return file_id, tag, key

        def get_id_from_str(self, id_str: str) -> SimID:
            ''' Returns a simulation id with the id in the string'''
            id_new = SimID()

            # split id types
            id_list = id_str.split(self.COMPONENT_SEPARATOR)

            for id_part in id_list:
                id_num = int(id_part[1:])

                if id_part[0] == self.pre_gid:
                    id_new.tru = id_num
                if id_part[0] == self.pre_mid:
                    id_new.mea = id_num
                if id_part[0] == self.pre_sid:
                    id_new.ser = id_num

            return id_new

        def is_filepath_valid(self, file_path: Path) -> bool:
            ''' Returns true if filename follows convention.

                id-tag-key.extension

            '''
            if file_path.is_dir():
                return False

            filename = str(file_path.name)

            split_extension = filename.split('.')
            split_field = split_extension[0].split(self.FIELD_SEPARATOR)

            if len(split_extension) != 2:
                return False
            if len(split_field) != 3:
                return False

            return True

        def get_filename(self, file_id: SimID, file_tag: str, file_key: str, extra='') -> str:
            ''' Returns filename according to convention.

                id-tag-key.extension

            '''
            file_id_str = self.get_file_id_str(file_id)

            file_name = str(
                file_id_str
                + self.FIELD_SEPARATOR
                + file_tag
                + self.FIELD_SEPARATOR
                + file_key
                + extra
            )

            return file_name


def remove_all_subfiles(path: Path):
    ''' Remove all subfiles and subfolders in a path. '''
    if not path.exists():
        path.mkdir()
        return  # TODO : add log message

    file_list = os.listdir(path.absolute())

    for ifile in file_list:
        # remove subfolder
        if Path(path, ifile).is_dir():
            remove_all_subfiles(Path(path, ifile).absolute())

        # remove file
        else:
            os.remove(Path(path, ifile).absolute())


def load_md_dict(fpath):
    new_dict = {}
    with open(fpath, newline='\n') as mdfile:
        line = mdfile.readline()
        while line != '':
            [key, value] = line.replace(' ', '').rstrip('\n').split(';')
            new_dict.update({key: value})
            line = mdfile.readline()

    return new_dict


