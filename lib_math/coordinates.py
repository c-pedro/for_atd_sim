import numpy as np

def spherical_to_orthogonal(radius, azimuth, zenith):
    ''' Returns the orthogonal coordinates of spherical coordinates.

        Reference
        ---------
        Arfken (1985, p. 102)

    '''
    x = radius * np.sin(zenith) * np.cos(azimuth)
    y = radius * np.sin(zenith) * np.sin(azimuth)
    z = radius * np.cos(zenith)

    return x,y,z
