import numpy as np
from .so3 import rot_mtx
from .coordinates import spherical_to_orthogonal

def random_unit_3d_vec():
    ''' Returns a random unit vector. '''
    azimuth = np.random.default_rng().random() * 2 * np.pi
    zenith = np.random.default_rng().random() * 2 * np.pi
    radius = 1

    # transform to orthogonal coordinates
    orth_arr = spherical_to_orthogonal(radius, azimuth, zenith)
    vec = np.array(orth_arr).reshape((3,1))

    # normalize and return
    return vec / np.linalg.norm(vec)


def random_atd():
    ''' Returns random attitude matrix. '''
    ang = 2 * np.pi * np.random.default_rng().random()
    axs = random_unit_3d_vec()

    return rot_mtx(ang, axs)
