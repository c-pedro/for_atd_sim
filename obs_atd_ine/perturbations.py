import numpy as np
import for_atd_sim.lib_math.so3 as so3

KEY_TYPE = 'var_type'

defSTD = [
    [np.pi/2, np.pi/4], # atd
    [np.pi/2, np.pi/4], # anv
    np.pi/4, # ref
    np.pi/4, # ilos
    [np.pi/6, np.pi/6] # man
]


def get_noises(*std):

    stdAtd1 = std[0][0]
    stdAtd2 = std[0][1]
    stdAnv1 = std[1][0]
    stdAnv2 = std[1][1]
    stdRef = std[2]
    stdIlos = std[3]
    stdMan1 = std[4][0]
    stdMan2 = std[4][1]

    atdNoise = np.array([
        [ [None], [stdAtd1, stdAtd2], [stdAtd1, stdAtd2], [stdAtd1, stdAtd2] ]
    ], object)

    anvNoise = np.array([
        [[None]],
        [ [stdAnv1, stdAnv2] ],
        [ [stdAnv1, stdAnv2] ],
        [ [stdAnv1, stdAnv2] ]
    ], object)

    refNoise = np.array([
        [[None]],
        [ [stdRef] ],
        [ [stdRef] ],
        [ [stdRef] ]
    ], object)

    ilosNoise = np.array([
        [ [None], [None], [None], [None]],
        [ [None], [None], [stdIlos], [stdIlos] ],
    ], object)

    manNoise = np.array([
        [ [None], [None], [None], [None]],
        [ [None], [None], [stdMan1, stdMan2], [stdMan1, stdMan2] ],
    ], object)

    return atdNoise, anvNoise, refNoise, ilosNoise, manNoise

def get_noisy_input(dataInput : dict, std =defSTD) -> dict:

    atdNoise, anvNoise, refNoise, ilosNoise, manNoise = get_noises(*std)

    noisyInput = {
        'pro_atd_value' : [(0,1), (0,2), (0,3)],
        'pro_anv_value' : [(1,0), (2,0), (3,0)],
        'pro_ref_value' : [(1,0), (2,0), (3,0)],
        'pro_ilos_value' : [(1,2), (1,3)],
        'pro_ilos_maneuver' : [(1,2)]
    }

    stdNoise = {
        'pro_atd_value' : atdNoise,
        'pro_anv_value' : anvNoise,
        'pro_ref_value' : refNoise,
        'pro_ilos_value' : ilosNoise,
        'pro_ilos_maneuver' : manNoise,

    }

    typeDB = {
        'pro_atd_value' : 'atd',
        'pro_anv_value' : 'vec',
        'pro_ref_value' : 'uvec',
        'pro_ilos_value' : 'uvec',
        'pro_ilos_maneuver' : 'man',
}

    # sample noisy values
    for ikey in noisyInput:
        for idx in noisyInput[ikey]:

            # sample variable
            newValue = sample_variable(
                                [dataInput[ikey][idx]],
                                stdNoise[ikey][idx],
                                **{
                                    KEY_TYPE : typeDB[ikey],
                                    'data' : dataInput
                                  }
                                )

            # update input
            dataInput[ikey][idx] = newValue

    return dataInput

def rand_unit_vector():
    mRng = np.random.default_rng()

    vec = (2 * mRng.random((3,1)) ) - np.ones((3,1))
    vec = vec / np.linalg.norm(vec)

    return vec


def sample_angle(nom, std):
    mRng = np.random.default_rng()

    return mRng.normal(nom, std)


def sample_axis(nom, stdAxisAn):

    an = sample_angle(0, stdAxisAn)
    ax = so3.skew_sym(np.array(nom)) @ rand_unit_vector()

    return so3.rot_mtx(an, ax) @ nom


def sample_vec(nom, stdAngle, stdScale ):

    newVec = sample_axis(nom, stdAngle)
    scale = sample_angle(0, stdScale)

    return scale * newVec


def sample_rotation(nomAxis, nomAngle, stdAxisAn, stdAngle):

    # sample new axis
    newAxis = sample_axis(nomAxis, stdAxisAn)
    # sample new angle
    newAngle = sample_angle(nomAngle, stdAngle)


    return so3.rot_mtx(newAngle, newAxis)

def sample_maneuver(nom : list, *std : list):
    newManList = []

    for man in nom:
        newMan = man.copy()
        newMan[2] = sample_angle(newMan[2], std[0])
        newMan[3] = sample_axis(newMan[3], std[1])
        newManList.append(newMan)

    return newManList

def sample_variable(nom : list, std : list, **kwargs):

    if KEY_TYPE in kwargs:
        vType = kwargs[KEY_TYPE]

    if vType == 'atd':
        nomAn, nomAx = get_rot_param(*nom)
        return sample_rotation(nomAx, nomAn, *std)
    elif vType == 'uvec':
        return sample_axis(*nom, *std)
    elif vType == 'vec':
        return sample_vec(*nom, *std)
    elif vType == 'ang':
        return sample_angle(*nom, *std)
    elif vType == 'man':
        return get_same_maneuver(*nom, kwargs['data'])
#         return sample_maneuver(*nom, *std)

def get_rot_param(rot):

    if np.trace(rot) > 3 - 10**-5:
        return 0, rand_unit_vector()

    an = np.arccos(0.5 * (np.trace(rot) - 1))

    ax = 1 / (2*np.sin(an)) * np.array([[rot[1,2]-rot[2,1]], [rot[2,0]-rot[0,2]], [rot[0,1]-rot[1,0]]])

    return an, ax

def get_same_maneuver(nom : list, data : dict):

    newManList = []

    for man in nom:
        newMan = man.copy()
        newMan[3] = data['pro_ilos_value'][1,2] + data['pro_ref_value'][2,0]
        newMan[3] = newMan[3] / np.linalg.norm(newMan[3])
        newManList.append(newMan)

    return newManList