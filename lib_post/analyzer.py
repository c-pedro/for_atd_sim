''' Post processing management.

'''
from pathlib import Path
from typing import List, Tuple
import matplotlib.pyplot as plt

PlotList = List[Tuple[List,List,dict]]

class Analyzer:

    def __init__(self) -> None:
        self.trf_function = callable

    def plot_data(self, fig_path: Path, plt_list : PlotList, **kwargs):
        ''' Plot data ina single figure. '''

        mfig = plt.figure()
        maxes = mfig.add_axes((0.1, 0.1, 0.9, 0.9))
        
        for plt_data in plt_list:        
            mlines = maxes.plot(plt_data[0], plt_data[1], **plt_data[2])
        
        maxes.set_yscale(kwargs.get('yscale','linear'))
        maxes.legend(kwargs.get('legend', []))

        print(fig_path)
        mfig.savefig(fig_path)
        plt.close(mfig)


    def transform_datalist(self, *trf_args):
        return self.trf_function(*trf_args)
            

    def transform_datadict(self, **trf_kwargs):
        return self.trf_function(**trf_kwargs)
        