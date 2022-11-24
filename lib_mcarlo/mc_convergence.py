from typing import List
from for_atd_sim.core.simid import SimID
from for_atd_sim.core.anlcell import AnlCell


def get_not_converged(key, index, time, id_list, limit, pkg_path) -> List[SimID]:

    acell : AnlCell = AnlCell(pkg_path)
    not_converged = []

    for sim_id in id_list:
        
        acell.id = sim_id
        acell.data.anl.update(**acell.load_results([key]))
        
        if not acell.check_convergence(key, index, time, limit):
            not_converged.append(sim_id)

    return not_converged