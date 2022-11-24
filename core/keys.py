''' Tools for key management, loading, and utilization. '''

# seprators
CHAR_SEP = '_'
FLD_SEP = '-'

# variables
KEY_ANG = 'angle'
KEY_ATD = 'atd'
KEY_ANV = 'anv'
KEY_LOS = 'los'
KEY_ILOS = 'ilos'
KEY_REF = 'ref'
KEY_POT = 'pot'
KEY_ANVERR = 'anverror'
KEY_PHI = 'phi'

# parameters
KEY_PAR = 'parameter'
KEY_PRECISE = 'precision'
KEY_IVP = 'ivp'
KEY_MOI = 'moi'
KEY_TOR = 'torque'
KEY_AXIS = 'axis'
KEY_TIME = 'key-time'
KEY_ITIME = 'initialtime'
KEY_FTIME = 'finaltime'
OPT_TIME_STEP = 'deltatime'
KEY_DATA = 'data'
KEY_INPUT = 'input'
KEY_OUTPUT = 'output'
KEY_MEAS = 'measurement'
KEY_NOISE = 'noise'
KEY_CONFIG = 'config'

# source
KEY_DB = 'database'
KEY_PRO = 'problem'
KEY_SEN = 'sensor'
KEY_OBSERVER = 'observer'

# fld keys
KEY_VAL = 'value'
KEY_COVAR = 'covar'
KEY_FNAME = 'filename'
KEY_SDEV = 'sdev'
KEY_PATH = 'path'

# comands
KEY_SAVE = 'save'
KEY_PLOT = 'plot'
KEY_MANV = 'maneuver'
KEY_CMD = 'command'

# types / origins
KEY_OPTION = 'option'

KEY_LOAD_DATA = 'loaddata'
KEY_INPUT_DATA = 'inputdata'

# time
KEY_TIME_INIT = 'time_initial'
KEY_TIME_FINAL = 'time_final'
KEY_TIME_STEP = 'time_step'
KEY_TIME_APPEND = 'time_append'

# file operations
KEY_KEEP_RES = 'results_keep'
KEY_CLEAR_RES = 'results_clear'
KEY_CLEAR_FIGS = 'results_figures_clear'

# file names
KEY_FNAME_SETT  = 'settings_filename'
KEY_FNAME_DATA  = 'data_filename'
KEY_FTAG_RES  = 'results_filename'

# directory names
KEY_DIRNAME_IN  = 'input_dirname'
KEY_DIRNAME_OUT  = 'ouput_dirname'

# paths
KEY_PATH_SPKG = 'dir_subpackage'
KEY_PATH_DATA = 'data_file'
KEY_PATH_KEYS = 'keys_file'
KEY_PATH_RES = 'results_file'
KEY_PATH_RES_LIST = 'results_list'

# commands
CMD_LIST = 'command_list_default'

# simulation
KEY_CON = 'continuous'
KEY_CNVG = 'converge'
KEY_APN = 'append'
KEY_DET = 'deterministic'

# id
KEY_IDM = 'idm'
KEY_SERIES_ID = 'simseries'
KEY_GROUND_ID = 'simground'
KEY_MEASUR_ID = 'simmeasure'


def get_key(*args) -> str:
    ''' Unified method of building keys'''
    my_key = ''
    for value in args:
        my_key += CHAR_SEP + value
    return my_key.lstrip(CHAR_SEP)


class KeyChain:
    ''' Base class for local module key management.

        The goal of the key chain is to give an id to each key. That id
        can then be used to bind together different local keys in the
        KeyManager.
    '''
    KEY_VAL = KEY_VAL
    KEY_COVAR = KEY_COVAR
    KEY_FNAME = KEY_FNAME
    KEY_SDEV = KEY_SDEV
    KEY_PATH = KEY_PATH

    @staticmethod
    def get_key(*args):
        return get_key(*args)


class ProKeys(KeyChain):
    ''' Manage deafult problem keys. '''

    prefix = 'pro' + CHAR_SEP

    # problem variables
    los = prefix + get_key(KEY_LOS, KEY_VAL)
    ilos = prefix + get_key(KEY_ILOS, KEY_VAL)
    ref = prefix + get_key(KEY_REF, KEY_VAL)
    atd = prefix + get_key(KEY_ATD, KEY_VAL)
    anv = prefix + get_key(KEY_ANV, KEY_VAL)

    pot = prefix + get_key(KEY_POT, KEY_VAL)
    pot_norm = prefix + get_key(KEY_POT, 'norm', KEY_VAL)

    # problem maneuvers
    manv = prefix + get_key(KEY_MANV)
    manv_ilos = prefix + get_key(KEY_ILOS, KEY_MANV)
    manv_ref = prefix + get_key(KEY_REF, KEY_MANV)

    # kinematic parameters
    par =  prefix + get_key(KEY_PAR, KEY_VAL)
    moi =  prefix + get_key(KEY_MOI, KEY_VAL)
    tor =  prefix + get_key(KEY_TOR, KEY_VAL)

    ivp_max_step = prefix + get_key('ivpmaxstep', KEY_VAL)

    # special cases metrics 
    metric_deg = prefix + get_key('metric', 'degenerate')
    metric_amb = prefix + get_key('metric', 'ambiguous')
    metric_cop = prefix + get_key('metric', 'coplanar')
    metric_lem = prefix + get_key('metric', 'lemma')




# sensor keychain
class SenKeys(KeyChain):

    prefix = 'sen' + CHAR_SEP

    # sensor variables
    los = prefix + get_key(KEY_LOS, KEY_VAL)
    ref = prefix + get_key(KEY_REF, KEY_VAL)
    anv = prefix + get_key(KEY_ANV, KEY_VAL)
    atd = prefix + get_key(KEY_ATD, KEY_VAL)

    # sensor covariances
    los_cov = prefix + get_key(KEY_LOS, KEY_COVAR)
    ref_cov = prefix + get_key(KEY_REF, KEY_COVAR)
    anv_cov = prefix + get_key(KEY_ANV, KEY_COVAR)

    # parameters
    par = prefix + get_key(KEY_PAR, KEY_VAL)

    # standard deviations
    los_sdev = prefix + get_key(KEY_LOS, KEY_SDEV)
    ref_sdev = prefix + get_key(KEY_REF, KEY_SDEV)
    anv_sdev = prefix + get_key(KEY_ANV, KEY_SDEV)



class ObsKeys(KeyChain):
    ''' Organize observer specific keys. '''

    prefix = 'obs' + CHAR_SEP

    # observer variables
    atd = prefix + get_key(KEY_ATD, KEY_VAL)
    atd_cov = prefix + get_key(KEY_ATD, KEY_COVAR)
    atd_sdv = prefix + get_key(KEY_ATD, KEY_SDEV)
    atd_sen = prefix + get_key('atd','sensor', KEY_VAL)

    # attitude errors
    atd_err = prefix + get_key('atderr', KEY_VAL)
    atd_err_norm = prefix + get_key('atderr','norm', KEY_VAL)
    atd_err_ang = prefix + get_key('atderr', 'angle', KEY_VAL)
    atd_err_ang_norm = prefix + get_key('atderr', 'angle', 'norm', KEY_VAL)
    mean_atd_err_ang_norm = prefix + get_key('mean','atderr', 'angle', 'norm', KEY_VAL)
    mean_conv_atd_err_ang_norm = prefix + get_key('mean', 'converged', 'atderr', 'angle', 'norm', KEY_VAL)
    atd_err_ang_sen = prefix + get_key(
        'atderr', 'angle', 'sensor', KEY_VAL
    )
    atd_err_ang_sen_norm = prefix + get_key(
        'atderr', 'angle', 'sensor', 'norm', KEY_VAL
    )
    atd_err_mean = prefix + get_key('atderr','mean', 'angle', KEY_VAL)

    # angular velocities
    anv = prefix + get_key(KEY_ANV, KEY_VAL)
    anv_err = prefix + get_key(KEY_ANVERR, KEY_VAL)
    anv_err_norm = prefix + get_key('anverr', 'norm', KEY_VAL)
    mean_anv_err_norm = prefix + get_key('mean','anverr', 'norm', KEY_VAL)
    mean_conv_anv_err_norm = prefix + get_key('mean', 'converged','anverr', 'norm', KEY_VAL)

    # observer obs_params
    par = prefix + get_key(KEY_PAR, KEY_VAL)

    # options 
    OPT_DET = prefix + get_key(KEY_DET, KEY_OPTION)

    # build keys
    KEY_MEAS = prefix + get_key(KEY_DB, KEY_SEN, KEY_VAL)
    KEY_PAR = prefix + get_key(KEY_PAR, KEY_SEN, KEY_VAL)
    KEY_OLD = prefix + get_key(KEY_DB, KEY_OBSERVER, KEY_VAL)

    # stability related
    KEY_LYAP = prefix + get_key('lyapunov', KEY_VAL)

    obs_damp = prefix + get_key('damping', KEY_VAL)
    weight_kin = prefix + get_key('wkinetic', KEY_VAL)
    weight_pot = prefix + get_key('wpotential', KEY_VAL)
