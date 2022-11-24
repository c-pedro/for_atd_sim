''' Compute statistics of variables. '''
from for_atd_sim.core.simid import SimID
from for_atd_sim.core.simcell import SimCell
from for_atd_sim.core.anlcell import AnlCell
from for_atd_sim.lib_data import DataSeries
from typing import List


def compute_mean(ser : int, mea :int, key : str, statskey : str, pkg_path):

    scell = SimCell(pkg_path)
    acell = AnlCell(pkg_path)

    scell.id.set_new_id(ser)

    mean_series = DataSeries() 
    size : int = 0

    acell.id.set_new_id(serid=ser, meaid=mea)
    last_truid = acell.fileman.find_last_grnid(acell.settings.get_path_output(), acell.id.ser)

    while acell.id.tru < last_truid.tru:
        size += 1
        next_id = acell.fileman.next_gid(acell.settings.get_path_output(), acell.id)
        acell.id = next_id
        acell.data.anl.update(**acell.load_results([key]))

        # sum results
        data_db = acell.data.get_specific_data([key])
        for i, idp in enumerate(data_db[key].series):
            
            if size == 1:
                mean_series.append_data(idp)
            else:
                for index in idp.data:
                    mean_series.series[i].data[index] += idp.data[index]


    # divide 
    for idp in mean_series.series:
        idp.data = {index:idp.data[index]/size for index in idp.data}

    # save stats series
    acell.fileman.save_csv_files(
        acell.settings.get_path_output(),
        SimID(acell.id.ser),
        {statskey : mean_series},
        acell.fileman.KEY_TAG_STATS
    )

    acell.data.anl.update({statskey : mean_series})
    


def compute_converged_mean(ser : int, mea :int, key : str, statskey : str, pkg_path, notconv_list : List[SimID]):
    scell = SimCell(pkg_path)
    acell = AnlCell(pkg_path)

    scell.id.set_new_id(ser)

    mean_series = DataSeries() 
    size : int = 0

    acell.id.set_new_id(serid=ser, meaid=mea)
    last_truid = acell.fileman.find_last_grnid(acell.settings.get_path_output(), acell.id.ser)
    # print(last_truid)

    while acell.id.tru < last_truid.tru:
        next_id = acell.fileman.next_gid(acell.settings.get_path_output(), acell.id)
        acell.id = next_id
        
        if acell.id in notconv_list:
            continue
        
        size += 1

        acell.data.anl.update(**acell.load_results([key]))

        # sum results
        data_db = acell.data.get_specific_data([key])
        for i, idp in enumerate(data_db[key].series):
            
            if size == 1:
                mean_series.append_data(idp)
            else:
                for index in idp.data:
                    mean_series.series[i].data[index] += idp.data[index]

    if size == 0:
        print('No trials converged on series {}'.format(ser))
        return

    # divide 
    for idp in mean_series.series:
        idp.data = {index:idp.data[index]/size for index in idp.data}

    # save stats series
    acell.fileman.save_csv_files(
        acell.settings.get_path_output(),
        SimID(acell.id.ser),
        {statskey : mean_series},
        acell.fileman.KEY_TAG_STATS
    )

    acell.data.anl.update({statskey : mean_series})