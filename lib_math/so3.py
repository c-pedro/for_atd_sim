''' Special orthogonal group related structures and algorithms.
'''
import numpy as np


def skew_sym(n : np.ndarray) -> np.ndarray:
    ''' Returns the skew symmetric matrix given by a column vector

        n : vector

    '''
    skew = np.array([
        [ 0      , -n[2,0],   n[1,0]],
        [ n[2,0],  0      ,  -n[0,0]],
        [-n[1,0],  n[0,0],   0      ]
        ])
    return skew


def inv_skew_sym(S : np.ndarray) -> np.ndarray:
    ''' Returns the vector of a skew-symmetric matrix.

        S : skew-symmetric matrix
    '''
    # ensure it is skew-symmetric
    np.testing.assert_array_almost_equal(S, -S.T)

    return np.array([[S[2,1]], [S[0,2]], [S[1,0]]])


def rot_mtx(a : float, n : np.ndarray) -> np.ndarray:
    ''' Returns a rotation matrix given an angle and an axis.
        a : angle
        n : axis

        Examples
        --------
        >>> rot_mtx(0, np.array([[1], [0], [0]]))
        ... # doctest: +NORMALIZE_WHITESPACE
        array([[1., 0., 0.],
                [0., 1., 0.],
                [0., 0., 1.]])

        >>> rot_mtx(np.pi/2, np.array([[1], [0], [0]]))
        ... # doctest: +NORMALIZE_WHITESPACE +SKIP
        array([[0,-1, 0],
                [1, 0, 0],
                [0, 0, 1]])
        [Approximately]

    '''
    # if angle is zero return the identity matrix
    if a == 0:
        return np.eye(3)

    # ensure axis is unit vector
    n = n / np.linalg.norm(n)

    # Rodrigues formula
    rotation = ( 
        + np.cos(a) * np.eye(3)
        + (1-np.cos(a)) * n @ n.T
        - np.sin(a) * skew_sym(n) 
    ) 

    return rotation


def orthogonalize_mat(mat : np.ndarray) -> np.ndarray:
    ''' Returns symmetric orthogonalization of a nonsingular matrix. 

        Parameters
        ----------
        mat : np.ndarray
            Nonsingular square matrix.

        Returns
        -------
        np.ndarray 
            Orthogonal matrix.

        Raises
        ------
        ValueError
            When mat is singular matrix.

    '''
    if abs(np.linalg.det(mat)) < 10**-10:
        raise ValueError

    u, _, vh = np.linalg.svd(mat)

    return u @ np.diag([1,1, np.linalg.det(u @ vh)]) @ vh