''' Manage analysis tasks.

'''
from pathlib import Path
from typing import Tuple, List

from for_atd_sim.core.simsettings import SimSettings
from for_atd_sim.core.simid import SimID
from for_atd_sim.core.simdata import SimData
from for_atd_sim.core.simcell import SimCell
from for_atd_sim.lib_io.file_manager import FileManager
from for_atd_sim.lib_post.analyzer import Analyzer
from for_atd_sim.lib_post.analyzer import PlotList


class AnlCell:
    ''' Manage a analysis tasks. '''

    def __init__(self, path_data: Path):
        ''' Initialize instance variables. '''
        self.settings = SimSettings(path_data)
        self.fileman = FileManager()
        self.id = SimID()
        self.data = SimData()
        self.analyzer = Analyzer()

    def set_from_simcell(self, scell: SimCell):
        self.settings = scell.settings
        self.fileman = scell.fileman
        self.data = scell.data
        self.id = scell.id

    def plot_data(self, plt_list: PlotList, fig_name: str, **kwargs):
        self.analyzer.plot_data(
            Path(self.fileman.figs_path(
                self.settings.get_path_output(), self.id.ser), fig_name),
            plt_list,
            **kwargs
        )

    def plot_var(self, key: str, index: Tuple[int, int], fig_name: str, kwargs_list: List[dict], **kwargs):
        tarr, darr = (self.data.res | self.data.anl)[
            key].get_plot_arrays(index)

        plt_list: PlotList = [
            (tarr, idarr, kwargs_list[i])
            for i, idarr in enumerate(darr)
        ]

        self.plot_data(plt_list, fig_name, **kwargs)

    def plot_multivar(self, key_index_list: List[Tuple[str, Tuple[int, int], dict[int, dict]]], fig_name: str, **other_options):
        plt_list: PlotList = []

        for (key, index, var_options) in key_index_list:

            tarr, darr = (self.data.res | self.data.anl)[
                key].get_plot_arrays(index)

            plt_list += [
                (tarr, idarr, var_options[i])
                for i, idarr in enumerate(darr)
            ]

        self.plot_data(plt_list, fig_name, **other_options)

    def transform_datalist(self, key_list: List[str], save_key: str, trf_function: callable):
        self.analyzer.trf_function = trf_function

        trf_args = self.data.get_data_list(key_list)
        self.data.anl[save_key] = self.analyzer.transform_datalist(*trf_args)
        self.save_anl_data(save_key)

    def transform_datadict(self, key_list: List[str], save_key: str, trf_function: callable):
        self.analyzer.trf_function = trf_function

        trf_kwargs = self.data.get_specific_data(key_list)
        self.data.anl[save_key] = self.analyzer.transform_datadict(
            **trf_kwargs)
        self.save_anl_data(save_key)

    def save_anl_data(self, save_key: str):
        self.fileman.save_csv_files(
            self.settings.get_path_output(),
            self.id,
            {save_key: self.data.anl[save_key]},
            self.fileman.KEY_TAG_ANL
        )

    def load_results(self, key_list: List[str]) -> dict:
        ''' Load and update results from file.'''
        load_data = {}
        for ikey in key_list:
            idata = self.fileman.load_single_file(
                self.settings.get_path_output(), self.id, ikey)
            load_data.update({ikey: idata})

        return load_data

    def check_convergence(self, key, index, time, limit) -> bool:
        dser_target = self.data.get_specific_data([key])[key]
        for dp in dser_target.series:
            if dp.time >= time:
                dp_target = dp
                break

        return dp_target.has_converged(index, limit)
