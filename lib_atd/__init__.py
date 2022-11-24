
from .error import compute_principal_angle
from .error import compute_error_rotation

from .general import model_dynamics_het

from .motion import get_formation_vectors

from .rigid_body import get_formation_dynamics
from .rigid_body import get_next_atd_dis

from .torque import tor_fun_db
from .torque import KEY_TOR_FUN

from .torque import KEY_TOR_PAR_INI
from .torque import KEY_TOR_PAR_GAIN
from .torque import KEY_TOR_PAR_FREQ
from .torque import KEY_TOR_PAR_PHASE

from .torque import KEY_TORFUN_STA
from .torque import KEY_TORFUN_SIN
from .torque import KEY_TORFUN_IMP

from .transform import trf_atd_error_list
from .transform import trf_atd_error_angle_list
from .transform import trf_max_atd_error_angle_list
from .transform import trf_anv_error_norm_list
from .transform import trf_max_anv_error_norm_list
from .transform import trf_orthogonalize_rotation_list