#!/usr/bin/env python3
''' Deterministic covariance.

    - TODO unit tests

'''
# third party
import numpy as np

# package
import for_atd_sim.obs_detatd_inerel.keys as kpkg
from for_atd_sim.lib_data.centralized_data import AttitudeData
from for_atd_sim.lib_data.centralized_data import DataPoint
from for_atd_sim.lib_math.so3 import skew_sym
from for_atd_sim.lib_math.so3 import rot_mtx
from for_atd_sim.obs_detatd_inerel.structures import InertiaslSolution


def get_principal_axis(size : int, i : int) -> np.ndarray:
    '''
        Return principal axis i of orthonormal coordinate frame.

        Parameters
        ----------
        size : int
            Coordinate frame dimension.
        i : int
            Axis number.

        Returns
        -------
        np.ndarray
            Principal axis requested.
    '''
    # initialize axis
    axis = np.zeros((size,1), float)
    # set unit value
    axis[i,0] = 1
    return axis



class CovarAtd():
    ''' Manage and compute attitude covariance. '''

    def __init__(self, sen_db = dict[str,DataPoint]) -> None:
        
        # initilize variables
        self.sigma = np.zeros(4)
        self.der_rotrel_d1 = np.empty((4), object)
        self.der_rotrel_d12 = np.empty((4), object)
        self.der_rotrel_d21 = np.empty((4), object)
        self.der_rotrel_d2 = np.empty((4), object)

        # initialize variables
        self.exp_rotrel = np.empty((4), object)
        self.exp_rotine_brn = np.empty((4), object)
        self.exp_rotine_brn_list = np.empty((4), object)

        # initilize gradients
        self.exp_state = np.empty((4), object)
        self.exp_cross = None
        self.grad_rel = np.empty((4), object)
        self.grad_ine = np.empty((4), object)

        # parse data points
        self.los_data : DataPoint = sen_db[kpkg.SenModelKeys.los]
        self.ref_data : DataPoint = sen_db[kpkg.SenModelKeys.ref]

        self.los_covar : DataPoint = sen_db[kpkg.SenModelKeys.los_cov]
        self.ref_covar : DataPoint = sen_db[kpkg.SenModelKeys.ref_cov]
        
        self.atd_data = AttitudeData(self.los_data.time)

        self.cov_pnt = DataPoint(self.los_data.time)
        self.sdev_pnt = DataPoint(self.los_data.time)


    def set_branch_solution(self, ine_sol : InertiaslSolution):

        # parse sigma
        self.sigma[2] = ine_sol.rot_12.aux.sign
        self.sigma[3] = ine_sol.rot_13.aux.sign

        # parse inertial candidates
        self.rot_ine_x = ine_sol.rot_x.rot
        self.rot_ine_y = ine_sol.rot_y.rot

        # set relative attitudes
        self.atd_data.data[(1,2)] = ine_sol.rot_12.rot
        self.atd_data.data[(1,3)] = ine_sol.rot_13.rot



    def compute_covariance_branches(self):
        ''' Compute relative and inertial covariances for both branches. '''

        # compute gradients and state expectations
        for idx in [2, 3]:
            try:
                self.grad_rel[idx] = self.gradient_rotrel_dvec(idx)
            except ValueError:
                aux = np.empty((3,3), object)
                for i in range(3):
                    for j in range(3):
                        aux[i,j] = np.full((1,12), np.nan)
                self.grad_rel[idx] = aux

            try:
                self.grad_ine[idx] = self.gradient_rotine_dvec(idx)
            except ValueError:
                aux = np.empty((3,3), object)
                for i in range(3):
                    for j in range(3):
                        aux[i,j] = np.full((1,12), np.nan)
                self.grad_ine[idx] = aux

        # get state covariance
        for idx in [2, 3]:
            self.exp_state[idx] = self.expected_matrix_rotrel(idx)

        self.exp_cross = self.expected_matrix_cross_branch()

        # compute covariance of each branch
        for idx in [2, 3]:
            self.exp_rotrel[idx] = self.compute_relative_covar(idx)
            self.exp_rotine_brn[idx] = self.compute_ine_branch_covar(idx)

        # compute covariance column-wise
        for idx in [2, 3]:
            self.exp_rotine_brn_list[idx] = self.get_ine_col_covar_list(idx)

        # compute principal angle covariance
        self.covar_ang_12 = self.compute_angles_covar(self.exp_rotrel[2])
        self.covar_ang_13 = self.compute_angles_covar(self.exp_rotrel[3])

        # append to point
        self.cov_pnt.data[(1,2)] = self.covar_ang_12
        self.cov_pnt.data[(1,3)] = self.covar_ang_13

        # append sdevs
        self.sdev_pnt.data[(1,2)] = self.compute_axis_sdev(self.covar_ang_12)
        self.sdev_pnt.data[(1,3)] = self.compute_axis_sdev(self.covar_ang_13)


    def compute_covariance_inertial(self, weights):
        ''' Compute inertial covariance of chief. '''

        # covariance of weighted procrustes
        self.covar_ang_i1 = self.compute_weighted_procrustes_covariance(weights)

        # append to point
        self.cov_pnt.data[(0,1)] = self.covar_ang_i1

        # append sdevs
        self.sdev_pnt.data[(0,1)] = self.compute_axis_sdev(self.covar_ang_i1)


    @staticmethod
    def compute_axis_sdev(axis_covar):
        return np.reshape(np.sqrt(np.diag(axis_covar)), (3,1))


    @staticmethod
    def compute_angles_covar(exp_val):
        return - (exp_val - 0.5 * np.trace(exp_val) * np.eye(3))


    @staticmethod
    def get_col_grad(grad, col):
        return np.concatenate(grad[:,col])


    @staticmethod
    def build_partial_matrix(der_rot_d1, der_rot_d12, der_rot_d21, der_rot_d2):
        der_rot_all = np.empty((3,3), object)

        for j in range(3):
            for k in range(3):
                der_rot_all[j,k] = np.array([
                    der_rot_d1[0][j,k],
                    der_rot_d1[1][j,k],
                    der_rot_d1[2][j,k],

                    der_rot_d12[0][j,k],
                    der_rot_d12[1][j,k],
                    der_rot_d12[2][j,k],

                    der_rot_d21[0][j,k],
                    der_rot_d21[1][j,k],
                    der_rot_d21[2][j,k],

                    der_rot_d2[0][j,k],
                    der_rot_d2[1][j,k],
                    der_rot_d2[2][j,k],
                ]).reshape((1, 12))
        return der_rot_all


    def compute_covar_rel_col(self, idx, col1, col2):

        grad1 = self.get_col_grad(self.grad_rel[idx], col1)
        grad2 = self.get_col_grad(self.grad_rel[idx], col2)

        exp = self.exp_state[idx]

        return grad1 @ exp @ grad2.T


    def compute_covar_ine_col(self, idx1, idx2, col1, col2):

        grad1 = self.get_col_grad(self.grad_ine[idx1], col1)
        grad2 = self.get_col_grad(self.grad_ine[idx2], col2)

        if idx1 == idx2:
            exp = self.exp_state[idx1]
        else:
            exp = self.exp_cross

        return grad1 @ exp @ grad2.T

    def compute_covar_finaline_col(self, col1, col2):

        left = (
            skew_sym(self.atd_data.data[0,1].T[:,col1].reshape((3,1))) 
            @
            self.atd_data.data[0,1].T
        )

        right = (
            skew_sym(self.atd_data.data[0,1].T[:,col2].reshape((3,1))) 
            @
            self.atd_data.data[0,1].T
        )

        return left @ self.covar_ang_i1 @ left.T 


    def compute_covar_rel_ine_col(self, idx2, col1, col2):
        idxr1 = (0,1)

        # left 0,1
        left = (
            skew_sym(self.atd_data.data[idxr1].T[:,col1].reshape((3,1))) 
            @ 
            self.atd_data.data[idxr1].T
        )

        # right rel
        grad_rel = self.get_col_grad(self.grad_rel[idx2], col2)

        exp_sum = np.zeros((3,12))
        for n in range(6):
            exp_sum += np.sqrt(self.weights[n])**(-2) * self.get_exp_cross_inerel(n, idx2)

        exp_sum = exp_sum / self.n_weight

        B = np.concatenate([np.eye(3), np.eye(3)], axis=1)
        exp_out = np.zeros((3,12))
        for m in range(6):
            exp_aux = -(
                np.sqrt(self.weights[m])**(-2) 
                * skew_sym(self.atd_data.data[0,1] @ B[:,m].reshape(3,1))
                @ (self.get_exp_cross_inerel(m, idx2) - exp_sum)
            )

            exp_out += exp_aux
        
        H = self.mat_h

        exp_out = np.linalg.inv(H) @ exp_out


        return left @ exp_out @ grad_rel.T


    # expected matrix
    def get_exp_cross_inerel(self, nidx, rel_idx):
        exp_in = np.zeros((3,3))
        
        if nidx in [0,1,2]:
            grad_ine = self.get_col_grad(self.grad_ine[2], nidx)
            exp_in = self.expected_matrix_rotrel(rel_idx)
        else:
            grad_ine = self.get_col_grad(self.grad_ine[3], nidx-3)
            exp_in = self.exp_cross

        return grad_ine @ exp_in


    def compute_relative_covar(self, idx=2):
        '''Compute covariance of relative covariance. '''
        # initialize covariance matrix
        my_cov = np.zeros((3,3), float)

        # sum column covariance
        for i in range(3):
            my_cov += self.compute_covar_rel_col(idx, i, i)

        return my_cov


    def get_ine_col_covar_list(self, idx=2):
        '''Compute covariance of inertial covariance. '''

        covar_list = []
        for col in range (3):
            covar_list.append(self.compute_covar_ine_col(idx, idx, col, col))

        return covar_list


    def compute_ine_branch_covar(self, idx=2):
        '''Compute covariance of inertial covariance. '''

        # initialize covariance matrix
        my_cov = np.zeros((3,3), float)

        for i in range(3):
            my_cov += self.compute_covar_ine_col(idx, idx, i, i)

        return my_cov


    def compute_procrustes_weights(self):
        # TODO not used?
        self.weight_list = []

        # branch 1-2 -> rx
        for i in range(3):
            self.weight_list.append(
                max(np.linalg.eigvals(
                    self.compute_covar_ine_col(2,2,i,i)
                ))
            )

        # branch 1-3 -> ry
        for i in range(3):
            self.weight_list.append(
                max(np.linalg.eigvals(
                    self.compute_covar_ine_col(3,3,i,i)
                ))
            )

        return self.weight_list


    def compute_vec_a(self, idx=2):

        d12 = self.los_data.data[1,idx]
        d21 = self.los_data.data[idx,1]

        return (d21-d12)/ np.linalg.norm(d21-d12)


    def compute_vec_b(self, idx=2):
        return -1 * self.los_data.data[1,idx]


    def compute_beta(self, idx=2):

        # parse data
        sigma = self.sigma[idx]

        # get coefficients
        cs, cc, cp = self.compute_coeff_b(idx)

        # check validity # TODO : deal with it
        if 1 - abs(cp / np.sqrt(cs**2 + cc**2 )) < 0:
            raise ValueError
        if cs**2 + cc**2 < 10**-10:
            raise ValueError

        # compute beta
        beta = (
            np.arctan2(cs,cc)
            + sigma * np.arccos(cp / np.sqrt(cs**2 + cc**2 ))
        )

        return beta


    def compute_coeff_b(self, idx=2):
        # parse data
        i1 = self.ref_data.data[1,0]
        i2 = self.ref_data.data[idx,0]
        d1 = self.ref_data.data[1,1]
        d2 = self.ref_data.data[idx,idx]
        d12 = self.los_data.data[1,idx]
        d21 = self.los_data.data[idx, 1]

        # get first rotation
        rot_a = self.compute_rot_a(idx)

        # compute coeficients
        cs = d1.T @ skew_sym(-d12) @ rot_a @ d2
        cc = d1.T @ skew_sym(-d12) @ skew_sym(-d12) @ rot_a @ d2
        cp = d1.T @ d12 * d12.T @ rot_a @ d2 - i1.T @ i2

        return cs, cc, cp


    def compute_rot_a(self, idx=2):
        ang = np.pi
        axs = self.compute_vec_a(idx)
        return rot_mtx(ang, axs)


    def compute_rot_b(self, idx=2):
        beta = self.compute_beta(idx)
        vec_b = self.compute_vec_b(idx)
        return rot_mtx(beta, vec_b)


    def gradient_a(self, idx=2) -> np.ndarray:
        ''' Return gradient of a.

            Parameters
            ----------
            idx : int, optional
                Branch index, by default 2

            Returns
            -------
            np.ndarray
                Gradient vector.
        '''
        # parse variables
        d12 = self.los_data.data[1, idx]
        d21 = self.los_data.data[idx, 1]

        # compute recurrent terms
        vec_y = d21 - d12
        norm_vecy = np.linalg.norm(vec_y)

        # initilize derivatives
        der_veca_d12 = np.empty((3,1), object)
        der_veca_d21 = np.empty((3,1), object)

        for i in range(3):
            prn_axs = get_principal_axis(3,i)

            der_veca_d12[i,0] = (
                (1 / norm_vecy)
                * ( vec_y * prn_axs.T @ vec_y / norm_vecy**2 - prn_axs)
            )

            der_veca_d21[i,0] = (
                (1 / norm_vecy)
                * ( prn_axs - vec_y * prn_axs.T @ vec_y / norm_vecy**2 )
            )

            np.testing.assert_almost_equal(
                der_veca_d12[i,0],  -der_veca_d21[i,0]
            )

        # return np.concatenate((der_veca_d12, der_veca_d21), axis=0)

        return np.concatenate([
            der_veca_d12[0,0].T,
            der_veca_d12[1,0].T,
            der_veca_d12[2,0].T,
            der_veca_d21[0,0].T,
            der_veca_d21[1,0].T,
            der_veca_d21[2,0].T
        ])


    def gradient_beta(self, idx=2) -> np.ndarray:
        ''' Return gradient of beta.

            Parameters
            ----------
            idx : int, optional
                Branch index, by default 2

            Returns
            -------
            np.ndarray
                Gradient vector.
        '''
        # sigma = 1  if A or C
        # sigma = -1  if B or D

        # parse sigma
        sigma = self.sigma[idx]

        # compute coeffcients
        cs,cc,cp = self.compute_coeff_b(idx)

        # compute denominators
        den0 = cs**2 + cc**2
        den1 = np.sqrt(cs**2 + cc**2 - cp**2)

        # compute partial derivatives
        der_beta_cs = (-cc + sigma * cs * cp / den1) / den0
        der_beta_cc = (-cs + sigma * cc * cp / den1) / den0
        der_beta_cp = - sigma / den1

        return np.concatenate([der_beta_cs, der_beta_cc, der_beta_cp])


    def gradient_rot_a(self, idx=2) -> np.ndarray:
        ''' Return partial derivatives of RA.

            Parameters
            ----------
            idx : int, optional
                Branch index, by default 2

            Returns
            -------
            np.ndarray
                Gradient vector.
        '''

        # compute vec_a and gradient
        vec_a = self.compute_vec_a(idx)
        grad_veca = self.gradient_a(idx)

        # initialize derivatives structured derivatives
        der_rota_d12 = np.empty((3,1), object)
        der_rota_d21 = np.empty((3,1), object)

        # d12i
        for i in range(3):
            # TODO iteration readability
            aux = np.reshape(grad_veca[i],(3,1)) @ vec_a.T
            der_rota_d12[i,0] = 2 * (
                (aux) + (aux).T
            )

            aux = np.reshape(grad_veca[i+3],(3,1)) @ vec_a.T
            der_rota_d21[i,0] = 2 * (
                (aux) + (aux).T
            )

        return der_rota_d12, der_rota_d21


    def compute_weighted_procrustes_covariance(self, in_weight) -> np.ndarray:
        '''
            Compute average inertial rotation covariance.

            Returns
            -------
            ndarray
                Error rotation axis covariance.

            Reference
            ---------
                Dorst, First order error propagation of the Procrustes method
                for 3D attitude estimation, 2005
        '''
        # parse variables
        rot_x = self.rot_ine_x
        rot_y = self.rot_ine_y
        rot = self.atd_data.data[0,1]

        # parse weight list
        weight_list = in_weight
        n_weight = (
                np.ones((6,1)).T
                @ np.linalg.inv(np.diag(weight_list))
                @ np.ones((6,1))
            )
        self.weights = in_weight
        self.n_weight = n_weight

        # compute observation covariance
        mat_y = np.concatenate([rot_x, rot_y], axis=1)
        mat_x = np.concatenate([np.eye(3), np.eye(3)], axis=1)

        vec_r = rot @ mat_x
        mat_a = np.zeros((3,3))
        for i in range(6):
            mat_a += (
                - weight_list[i]**-1
                * skew_sym(vec_r[:,i].reshape((3,1)))
                @ skew_sym(vec_r[:,i].reshape((3,1))).T
            )
            for j in range(6):
                mat_a += (
                    weight_list[i]**-1
                    * weight_list[j]**-1
                    * skew_sym(vec_r[:,i].reshape((3,1)))
                    @ skew_sym(vec_r[:,j].reshape((3,1))).T
                    / n_weight
                )

        # compute all a_bar expected
        exp_a_bar = np.empty((6,6), object)
        for i in range(6):
            for j in range(6):
                exp_a_bar[i,j] = np.zeros((3,3))

        for i in range(6):
            for j in range(6):
                exp_a_bar[i,j] += self.get_expected_a(i,j)
                for k in range(6):
                    exp_a_bar[i,j] += (
                        - weight_list[k]**-1
                        * self.get_expected_a(i,k)
                        / n_weight
                    )

                    exp_a_bar[i,j] += (
                        - weight_list[k]**-1
                        * self.get_expected_a(k,j)
                        / n_weight
                    )

                    for l in range(6):
                        exp_a_bar[i,j] += (
                            weight_list[k]**-1
                            * weight_list[l]**-1
                            * self.get_expected_a(k,l)
                            / n_weight**2
                        )

        # compute average covariance
        cov_da = np.zeros((3,3))
        for i in range(mat_y.shape[1]):
            for j in range(mat_y.shape[1]):
                cov_da += np.linalg.inv(mat_a) @ (
                    weight_list[i]**-1
                    * weight_list[j]**-1
                    * skew_sym(vec_r[:,i].reshape((3,1)))
                    @ exp_a_bar[i,j]
                    @ skew_sym(vec_r[:,j].reshape((3,1))).T
                ) @ np.linalg.inv(mat_a).T
        # save H 
        self.mat_h = mat_a

        return cov_da


    def get_expected_a(self, col1 : int, col2 : int):

        # assess first index
        if col1 <3:
            idx_1 = 2
        else:
            col1 -= 3
            idx_1 = 3

        # assess second index
        if col2 <3:
            idx_2 = 2
        else:
            col2 -= 3
            idx_2 = 3

        return self.compute_covar_ine_col(idx_1, idx_2, col1, col2)


    # single matrix approach
    def compute_k1_k2(self, idx=2):
        # parse variables
        i1 = self.ref_data.data[1,0]
        i2 = self.ref_data.data[idx,0]

        k1 = skew_sym(i1) @ i2 / np.linalg.norm(skew_sym(i1) @ i2)
        k2 = skew_sym(i1) @ k1

        return k1, k2


    def expected_matrix_rotrel(self, idx=2):

        # parse measurement covariance
        cov_d1 = self.ref_covar.data[(1,1)]
        cov_d2 = self.ref_covar.data[(idx,idx)]
        cov_d12 = self.los_covar.data[(1,idx)]
        cov_d21 = self.los_covar.data[(idx,1)]



        # build block matrix
        exp_block = np.block([
            [cov_d1, np.zeros((3, 9))],
            [np.zeros((3, 3)), cov_d12, np.zeros((3, 6))],
            [np.zeros((3, 6)), cov_d21, np.zeros((3, 3))],
            [np.zeros((3, 9)), cov_d2]
        ])

        return exp_block


    def expected_matrix_cross_branch(self):

        # parse measurement covariance
        cov_d1 = self.ref_covar.data[(1,1)]

        # build block matrix
        exp_block = np.block([
            [cov_d1, np.zeros((3, 9))],
            [np.zeros((3, 12))],
            [np.zeros((3, 12))],
            [np.zeros((3, 12))]
        ])

        return exp_block


    def gradient_rotrel_dvec(self, idx=2):
        ''' Return gradient matrix of relative rotation. '''

        # get rotation partial derivatives
        der_rot_veca = self.partial_rot_veca(idx)
        der_rot_vecb = self.partial_rot_vecb(idx)
        der_rot_beta = self.partial_rot_beta(idx)

        # get vec a partial derivatives
        der_veca_d12 = self.partial_veca_d12(idx)
        der_veca_d21 = self.partial_veca_d21(idx)

        # get vec b partial derivatives
        der_vecb_d12 = self.partial_vecb_d12(idx)

        # get beta partial derivatives
        der_beta_cs = self.partial_beta_cs(idx)
        der_beta_cc = self.partial_beta_cc(idx)
        der_beta_cp = self.partial_beta_cp(idx)

        # get cs partial derivatives
        der_cs_d1 = self.partial_cs_d1(idx)
        der_cs_d12 = self.partial_cs_d12(idx)
        der_cs_d21 = self.partial_cs_d21(idx)
        der_cs_d2 = self.partial_cs_d2(idx)

        # get cc partial derivatives
        der_cc_d1 = self.partial_cc_d1(idx)
        der_cc_d12 = self.partial_cc_d12(idx)
        der_cc_d21 = self.partial_cc_d21(idx)
        der_cc_d2 = self.partial_cc_d2(idx)

        # get cp partial derivatives
        der_cp_d1 = self.partial_cp_d1(idx)
        der_cp_d12 = self.partial_cp_d12(idx)
        der_cp_d21 = self.partial_cp_d21(idx)
        der_cp_d2 = self.partial_cp_d2(idx)

        # compute partial derivative of rotation relative to measurements
        # initialize
        der_rot_d1 = []
        der_rot_d2 = []
        der_rot_d12 = []
        der_rot_d21 = []

        for j in range(3):
            # append partial to d1
            der_rot_d1.append(
                der_rot_beta * (
                    der_beta_cs * der_cs_d1[j]
                    + der_beta_cc * der_cc_d1[j]
                    + der_beta_cp * der_cp_d1[j]
                )
            )

            # sum partial to veca and vecb
            sum_veca_vecb_d12 = np.zeros((3,3), float)
            for k in range(3):
                sum_veca_vecb_d12 += (
                    der_rot_veca[k] * der_veca_d12[j][k,0]
                    + der_rot_vecb[k] * der_vecb_d12[j][k,0]
                )

            # append partial to d12
            der_rot_d12.append(
                sum_veca_vecb_d12
                + der_rot_beta * (
                    der_beta_cs * der_cs_d12[j]
                    + der_beta_cc * der_cc_d12[j]
                    + der_beta_cp * der_cp_d12[j]
                )
            )

            # sum partial to veca and vecb
            sum_veca_d21 = np.zeros((3,3), float)
            for k in range(3):
                sum_veca_d21 += (
                    der_rot_veca[k] * der_veca_d21[j][k,0]
                )

            # append partial to d21
            der_rot_d21.append(
                sum_veca_d21
                + der_rot_beta * (
                    der_beta_cs * der_cs_d21[j]
                    + der_beta_cc * der_cc_d21[j]
                    + der_beta_cp * der_cp_d21[j]
                )
            )

            # append partial to d2
            der_rot_d2.append(
                der_rot_beta * (
                    der_beta_cs * der_cs_d2[j]
                    + der_beta_cc * der_cc_d2[j]
                    + der_beta_cp * der_cp_d2[j]
                )
            )

        # save vars
        self.der_rotrel_d1[idx] = der_rot_d1
        self.der_rotrel_d2[idx] = der_rot_d2
        self.der_rotrel_d12[idx] = der_rot_d12
        self.der_rotrel_d21[idx] = der_rot_d21

        return self.build_partial_matrix(
            der_rot_d1, der_rot_d12,
            der_rot_d21, der_rot_d2
        )


    def gradient_rotine_dvec(self, idx=2):
        ''' Return gradient matrix of relative rotation. '''

        # get rotation partial derivatives
        der_rot_veca = self.partial_rotine_veca(idx)
        der_rot_vecb = self.partial_rotine_vecb(idx)
        der_rot_beta = self.partial_rotine_beta(idx)

        # get vec a partial derivatives
        der_veca_d12 = self.partial_veca_d12(idx)
        der_veca_d21 = self.partial_veca_d21(idx)

        # get vec b partial derivatives
        der_vecb_d12 = self.partial_vecb_d12(idx)

        # get beta partial derivatives
        der_beta_cs = self.partial_beta_cs(idx)
        der_beta_cc = self.partial_beta_cc(idx)
        der_beta_cp = self.partial_beta_cp(idx)

        # get cs partial derivatives
        der_cs_d1 = self.partial_cs_d1(idx)
        der_cs_d12 = self.partial_cs_d12(idx)
        der_cs_d21 = self.partial_cs_d21(idx)
        der_cs_d2 = self.partial_cs_d2(idx)

        # get cc partial derivatives
        der_cc_d1 = self.partial_cc_d1(idx)
        der_cc_d12 = self.partial_cc_d12(idx)
        der_cc_d21 = self.partial_cc_d21(idx)
        der_cc_d2 = self.partial_cc_d2(idx)

        # get cp partial derivatives
        der_cp_d1 = self.partial_cp_d1(idx)
        der_cp_d12 = self.partial_cp_d12(idx)
        der_cp_d21 = self.partial_cp_d21(idx)
        der_cp_d2 = self.partial_cp_d2(idx)

        der_rotine_d1 = self.partial_rotine_d1(idx)
        der_rotine_d2 = self.partial_rotine_d2(idx)

        # compute partial derivative of rotation relative to measurements
        # initialize
        der_rot_d1 = []
        der_rot_d2 = []
        der_rot_d12 = []
        der_rot_d21 = []

        for j in range(3):
            # append partial to d1
            der_rot_d1.append(
                der_rotine_d1[j]
                + der_rot_beta * (
                    der_beta_cs * der_cs_d1[j]
                    + der_beta_cc * der_cc_d1[j]
                    + der_beta_cp * der_cp_d1[j]
                )
            )

            # sum partial to veca and vecb
            sum_veca_vecb_d12 = np.zeros((3,3), float)
            for k in range(3):
                sum_veca_vecb_d12 += (
                    der_rot_veca[k] * der_veca_d12[j][k,0]
                    + der_rot_vecb[k] * der_vecb_d12[j][k,0]
                )

            # append partial to d12
            der_rot_d12.append(
                sum_veca_vecb_d12
                + der_rot_beta * (
                    der_beta_cs * der_cs_d12[j]
                    + der_beta_cc * der_cc_d12[j]
                    + der_beta_cp * der_cp_d12[j]
                )
            )

            # sum partial to veca and vecb
            sum_veca_d21 = np.zeros((3,3), float)
            for k in range(3):
                sum_veca_d21 += (
                    der_rot_veca[k] * der_veca_d21[j][k,0]
                )

            # append partial to d21
            der_rot_d21.append(
                sum_veca_d21
                + der_rot_beta * (
                    der_beta_cs * der_cs_d21[j]
                    + der_beta_cc * der_cc_d21[j]
                    + der_beta_cp * der_cp_d21[j]
                )
            )

            # append partial to d2
            der_rot_d2.append(
                der_rotine_d2[j]
                + der_rot_beta * (
                    der_beta_cs * der_cs_d2[j]
                    + der_beta_cc * der_cc_d2[j]
                    + der_beta_cp * der_cp_d2[j]
                )
            )

        # concatenate all partial derivatives
        return self.build_partial_matrix(
            der_rot_d1, der_rot_d12,
            der_rot_d21, der_rot_d2
        )


    def partial_rotine_d1(self, idx=2):
        ''' Return partial derivative of inertial rotation to explicit d1. '''
        # initialize list
        der_rot_d1 = []

        # parse variables
        i1 = self.ref_data.data[1,0]
        d1 = self.ref_data.data[1,1]
        d2 = self.ref_data.data[idx,idx]
        rot12 = self.atd_data.data[1,idx]

        # compute auxiliary variables
        k1, k2 = self.compute_k1_k2(idx)
        term_1 = skew_sym(d1) @ rot12 @ d2

        for j in range(3):
            prn_axs = get_principal_axis(3,j)
            der_norm_vec = (
                term_1.T
                @ (
                    skew_sym(prn_axs) @ rot12 @ d2
                ) / np.linalg.norm(term_1)
            )

            der_rot_d1.append(
                i1 @ prn_axs.T
                + k1 @ (
                    skew_sym(prn_axs) @ rot12 @ d2
                    / np.linalg.norm(term_1)
                    - term_1 @ der_norm_vec
                    / np.linalg.norm(term_1)**2
                ).T
                + k2 @ (
                    skew_sym(prn_axs) @ skew_sym(d1) @ rot12 @ d2
                    / np.linalg.norm(term_1)
                    + skew_sym(d1) @ skew_sym(prn_axs) @ rot12 @ d2
                    / np.linalg.norm(term_1)
                    - skew_sym(d1) @ term_1 @ der_norm_vec
                    / np.linalg.norm(term_1)**2
                ).T
            )

        return der_rot_d1


    def partial_rotine_d2(self, idx=2):
        ''' Return partial derivative of inertial rotation to vec a. '''
        # initialize list
        der_rot_d2 = []

        # parse variables
        d1 = self.ref_data.data[1,1]
        d2 = self.ref_data.data[idx,idx]
        rot12 = self.atd_data.data[1,idx]

        # compute auxiliary variables
        k1, k2 = self.compute_k1_k2(idx)
        term_1 = skew_sym(d1) @ rot12 @ d2

        for j in range(3):
            prn_axs = get_principal_axis(3,j)

            der_norm_vec = (
                term_1.T
                @ (
                    skew_sym(d1) @ rot12 @ prn_axs
                )
                / np.linalg.norm(term_1)
            )

            der_rot_d2.append(
                + k1 @ (
                    skew_sym(d1) @ rot12 @ prn_axs
                    / np.linalg.norm(term_1)
                    - term_1 @ der_norm_vec
                    / np.linalg.norm(term_1)**2
                ).T
                + k2 @ (
                    skew_sym(d1) @ skew_sym(d1) @ rot12 @ prn_axs
                    / np.linalg.norm(term_1)
                    - skew_sym(d1) @ term_1 @ der_norm_vec
                    / np.linalg.norm(term_1)**2
                ).T
            )

        return der_rot_d2


    def partial_rotine_veca(self, idx=2):
        ''' Return partial derivative of inertial rotation to vec a. '''
        # initialize list
        der_rot_veca = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d1 = self.ref_data.data[1,1]
        d2 = self.ref_data.data[idx,idx]
        rot12 = self.atd_data.data[1,idx]

        # TODO : change name to rotrel
        der_rot12_veca = self.partial_rot_veca(idx) # TODO : no repeat

        # compute auxiliary variables
        k1, k2 = self.compute_k1_k2(idx)
        term_1 = skew_sym(d1) @ rot12 @ d2

        for j in range(3):
            der_norm_vec = (
                term_1.T
                @ skew_sym(d1) @ der_rot12_veca[j] @ d2
                / np.linalg.norm(term_1)
            )

            der_rot_veca.append(
                k1 @ (
                    skew_sym(d1) @ der_rot12_veca[j] @ d2
                    / np.linalg.norm(term_1)
                    - term_1 @ der_norm_vec
                    / np.linalg.norm(term_1)**2
                ).T
                + k2 @ (
                    skew_sym(d1) @ skew_sym(d1) @ der_rot12_veca[j] @ d2
                    / np.linalg.norm(term_1)
                    - skew_sym(d1) @ term_1 @ der_norm_vec
                    / np.linalg.norm(term_1)**2
                ).T
            )

        return der_rot_veca


    def partial_rotine_vecb(self, idx=2):
        ''' Return partial derivative of inertial rotation to vec b. '''
        # initialize list
        der_rot_vecb = []

        # parse variables
        d1 = self.ref_data.data[1,1]
        d2 = self.ref_data.data[idx,idx]
        rot12 = self.atd_data.data[1,idx]

        # TODO : no repeat
        # TODO : change name to rotrel
        der_rot12_vecb = self.partial_rot_vecb(idx)

        # compute auxiliary variables
        k1, k2 = self.compute_k1_k2(idx)
        term_1 = skew_sym(d1) @ rot12 @ d2

        for j in range(3):
            der_norm_vec = (
                term_1.T
                @ skew_sym(d1) @ der_rot12_vecb[j] @ d2
                / np.linalg.norm(term_1)
            )

            der_rot_vecb.append(
                k1 @ (
                    skew_sym(d1) @ der_rot12_vecb[j] @ d2
                    / np.linalg.norm(term_1)
                    - term_1 @ der_norm_vec
                    / np.linalg.norm(term_1)**2
                ).T
                + k2 @ (
                    skew_sym(d1) @ skew_sym(d1) @ der_rot12_vecb[j] @ d2
                    / np.linalg.norm(term_1)
                    - skew_sym(d1) @ term_1 @ der_norm_vec
                    / np.linalg.norm(term_1)**2
                ).T
            )

        return der_rot_vecb


    def partial_rotine_beta(self, idx=2):
        ''' Return partial derivative of inertial rotation to beta. '''
        # initialize list
        der_rot_beta = []

        # parse variables
        d1 = self.ref_data.data[1,1]
        d2 = self.ref_data.data[idx,idx]
        rot12 = self.atd_data.data[1,idx]

        # TODO : no repeat
        # TODO : change name to rotrel
        der_rot12_beta = self.partial_rot_beta(idx)

        # compute auxiliary variables
        k1, k2 = self.compute_k1_k2(idx)
        term_1 = skew_sym(d1) @ rot12 @ d2

        der_norm_vec = (
            term_1.T
            @ skew_sym(d1) @ der_rot12_beta @ d2
            / np.linalg.norm(term_1)
        )

        der_rot_beta = (
            k1 @ (
                skew_sym(d1) @ der_rot12_beta @ d2
                / np.linalg.norm(term_1)
                - term_1 @ der_norm_vec
                / np.linalg.norm(term_1)**2
            ).T
            + k2 @ (
                skew_sym(d1) @ skew_sym(d1) @ der_rot12_beta @ d2
                / np.linalg.norm(term_1)
                - skew_sym(d1) @ term_1 @ der_norm_vec
                / np.linalg.norm(term_1)**2
            ).T
        )

        return der_rot_beta


    def partial_rot_veca(self, idx=2):
        ''' Return partial derivative of relative rotation to vec a. '''
        # initialize list
        der_rot_veca = []

        # parse variables
        veca = self.compute_vec_a(idx)
        vecb = self.compute_vec_b(idx)
        beta = self.compute_beta(idx)

        for j in range(3):
            prn_axs = get_principal_axis(3, j)
            der_rot_veca.append(
                2 * rot_mtx(beta, vecb) @ (prn_axs @ veca.T + veca @ prn_axs.T)
            )

        return der_rot_veca


    def partial_rot_vecb(self, idx=2):
        ''' Return partial derivative of relative rotation to vec b. '''
        # initialize list
        der_rot_vecb = []

        # parse variables
        veca = self.compute_vec_a(idx)
        vecb = self.compute_vec_b(idx)
        beta = self.compute_beta(idx)

        for j in range(3):
            prn_axs = get_principal_axis(3, j)
            der_rot_vecb.append(
                (
                    (1- np.cos(beta)) * (prn_axs @ vecb.T + vecb @ prn_axs.T)
                    - np.sin(beta) * skew_sym(prn_axs)
                )
                @ rot_mtx(np.pi, veca)
            )

        return der_rot_vecb


    def partial_rot_beta(self, idx=2):
        ''' Return partial derivative of relative rotation to beta. '''
        # parse variables
        vecb = self.compute_vec_b(idx)
        beta = self.compute_beta(idx)
        rota = self.compute_rot_a(idx)

        der_rot_beta = (
                (
                    np.sin(beta) * ( vecb @ vecb.T - np.eye(3) )
                    - np.cos(beta) * skew_sym(vecb)
                )
                @ rota
            )

        return der_rot_beta


    def partial_veca_d12(self, idx=2):
        ''' Return partial derivative of veca to d12. '''
        # initialize list
        der_veca_d12 = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d12 = self.los_data.data[1,idx]
        d21 = self.los_data.data[idx,1]

        for j in range(3):
            prn_axs = get_principal_axis(3, j)
            der_veca_d12.append(
                (veca @ veca.T - np.eye(3)) @ prn_axs / np.linalg.norm(d21-d12)
            )

        return der_veca_d12


    def partial_veca_d21(self, idx=2):
        ''' Return partial derivative of veca to d21. '''
        # initialize list
        der_veca_d21 = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d12 = self.los_data.data[1,idx]
        d21 = self.los_data.data[idx,1]

        for j in range(3):
            prn_axs = get_principal_axis(3, j)
            der_veca_d21.append(
                (np.eye(3) - veca @ veca.T) @ prn_axs / np.linalg.norm(d21-d12)
            )

        return der_veca_d21


    def partial_vecb_d12(self, idx=2):
        ''' Return partial derivative of vecb to d12. '''
        # initialize list
        der_vecb_d12 = []

        for j in range(3):
            prn_axs = get_principal_axis(3, j)
            der_vecb_d12.append(-prn_axs)

        return der_vecb_d12


    def partial_beta_cs(self, idx=2):
        ''' Return partial derivative of beta to cs. '''
        # parse variables
        sigma = self.sigma[idx]
        cs, cc, cp  = self.compute_coeff_b(idx)

        if cs**2 + cc**2 - cp**2 < 10**-10:
            raise ValueError

        der_beta_cs = (
            (cc + sigma * cs * cp / np.sqrt(cs**2 + cc**2 - cp**2))
            / (cs**2 + cc**2)
        )

        return der_beta_cs


    def partial_beta_cc(self, idx=2):
        ''' Return partial derivative of beta to cc. '''
        # parse variables
        sigma = self.sigma[idx]
        cs, cc, cp  = self.compute_coeff_b(idx)

        # check validity
        if cs**2 + cc**2 - cp**2 < 10**-10:
            raise ValueError

        # compute derivative
        der_beta_cc = (
            (-cs + sigma * cc * cp / np.sqrt(cs**2 + cc**2 - cp**2))
            / (cs**2 + cc**2)
        )

        return der_beta_cc


    def partial_beta_cp(self, idx=2):
        ''' Return partial derivative of beta to cp. '''
        # parse variables
        sigma = self.sigma[idx]
        cs, cc, cp  = self.compute_coeff_b(idx)

        der_beta_cp = - sigma / np.sqrt(cs**2 + cc**2 - cp**2)

        return der_beta_cp


    def partial_cs_d1(self, idx=2):
        ''' Return partial derivative of cs to d1. '''
        # initialize list
        der_cs_d1 = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d2 = self.ref_data.data[idx,idx]
        d12 = self.los_data.data[1,idx]

        for j in range(3):
            prn_axs = get_principal_axis(3, j)
            der_cs_d1.append(
                prn_axs.T @ skew_sym(-d12) @ rot_mtx(np.pi, veca) @ d2
            )

        return der_cs_d1


    def partial_cs_d12(self, idx=2):
        ''' Return partial derivative of cs to d12. '''
        # initialize list
        der_cs_d12 = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d1 = self.ref_data.data[1,1]
        d2 = self.ref_data.data[idx,idx]
        d12 = self.los_data.data[1,idx]
        der_veca_d12 = self.partial_veca_d12(idx)

        for j in range(3):
            prn_axs = get_principal_axis(3, j)
            der_cs_d12.append(
                d1.T @ skew_sym(-prn_axs) @ rot_mtx(np.pi, veca) @ d2
                + 2 * d1.T @ skew_sym(-d12) @ (
                    der_veca_d12[j] @ veca.T + veca @ der_veca_d12[j].T
                ) @ d2
            )

        return der_cs_d12


    def partial_cs_d21(self, idx=2):
        ''' Return partial derivative of cs to d21. '''
            # initialize list
        der_cs_d21 = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d1 = self.ref_data.data[1,1]
        d2 = self.ref_data.data[idx,idx]
        d12 = self.los_data.data[1,idx]
        der_veca_d21 = self.partial_veca_d21(idx)

        for j in range(3):
            der_cs_d21.append(
                + 2 * d1.T @ skew_sym(-d12) @ (
                    der_veca_d21[j] @ veca.T + veca @ der_veca_d21[j].T
                ) @ d2
            )

        return der_cs_d21


    def partial_cs_d2(self, idx=2):
        ''' Return partial derivative of cs to d2. '''
        # initialize list
        der_cs_d2 = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d1 = self.ref_data.data[1,1]
        d12 = self.los_data.data[1,idx]

        for j in range(3):
            prn_axs = get_principal_axis(3, j)
            der_cs_d2.append(
                d1.T @ skew_sym(-d12) @ rot_mtx(np.pi, veca) @ prn_axs
            )

        return der_cs_d2


    def partial_cc_d1(self, idx=2):
        ''' Return partial derivative of cc to d1. '''
       # initialize list
        der_cc_d1 = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d2 = self.ref_data.data[idx,idx]
        d12 = self.los_data.data[1,idx]

        for j in range(3):
            prn_axs = get_principal_axis(3, j)
            der_cc_d1.append(
                prn_axs.T @ skew_sym(d12) @ skew_sym(d12)
                @ rot_mtx(np.pi, veca) @ d2
            )

        return der_cc_d1


    def partial_cc_d12(self, idx=2):
        ''' Return partial derivative of cc to d12. '''
        # initialize list
        der_cc_d12 = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d1 = self.ref_data.data[1,1]
        d2 = self.ref_data.data[idx,idx]
        d12 = self.los_data.data[1,idx]
        der_veca_d12 = self.partial_veca_d12(idx)

        # compute d2star
        dstar = rot_mtx(np.pi, veca) @ d2

        for j in range(3):
            prn_axs = get_principal_axis(3, j)
            der_cc_d12.append(
                d1.T @ skew_sym(prn_axs) @ skew_sym(d12) @ dstar
                + d1.T @ skew_sym(d12) @ skew_sym(prn_axs) @ dstar
                + 2 * d1.T @ skew_sym(d12) @ skew_sym(d12) @ (
                    der_veca_d12[j] @ veca.T + veca @ der_veca_d12[j].T
                ) @ d2
            )

        return der_cc_d12


    def partial_cc_d21(self, idx=2):
        ''' Return partial derivative of cc to d21. '''
        # initialize list
        der_cc_d21 = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d1 = self.ref_data.data[1,1]
        d2 = self.ref_data.data[idx,idx]
        d12 = self.los_data.data[1,idx]
        der_veca_d21 = self.partial_veca_d21(idx)

        for j in range(3):
            der_cc_d21.append(
                + 2 * d1.T @ skew_sym(d12) @ skew_sym(d12) @ (
                    der_veca_d21[j] @ veca.T + veca @ der_veca_d21[j].T
                ) @ d2
            )

        return der_cc_d21


    def partial_cc_d2(self, idx=2):
        ''' Return partial derivative of cc to d2. '''
       # initialize list
        der_cc_d2 = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d1 = self.ref_data.data[1,1]
        d12 = self.los_data.data[1,idx]

        for j in range(3):
            prn_axs = get_principal_axis(3, j)
            der_cc_d2.append(
                d1.T @ skew_sym(d12) @ skew_sym(d12)
                @ rot_mtx(np.pi, veca) @ prn_axs
            )

        return der_cc_d2


    def partial_cp_d1(self, idx=2):
        ''' Return partial derivative of cp to d1. '''
       # initialize list
        der_cp_d1 = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d2 = self.ref_data.data[idx,idx]
        d12 = self.los_data.data[1,idx]

        for j in range(3):
            prn_axs = get_principal_axis(3, j)
            der_cp_d1.append(
                prn_axs.T @ d12 @ d12.T @ rot_mtx(np.pi, veca) @ d2
            )

        return der_cp_d1


    def partial_cp_d12(self, idx=2):
        ''' Return partial derivative of cp to d12. '''
        # initialize list
        der_cp_d12 = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d1 = self.ref_data.data[1,1]
        d2 = self.ref_data.data[idx,idx]
        d12 = self.los_data.data[1,idx]
        der_veca_d12 = self.partial_veca_d12(idx)

        # compute d2star
        dstar = rot_mtx(np.pi, veca) @ d2

        for j in range(3):
            prn_axs = get_principal_axis(3, j)
            der_cp_d12.append(
                d1.T @ prn_axs @ d12.T @ dstar
                + d1.T @ d12 @ prn_axs.T @ dstar
                + 2 * d1.T @ d12 @ d12.T @ (
                    der_veca_d12[j] @ veca.T + veca @ der_veca_d12[j].T
                ) @ d2
            )

        return der_cp_d12


    def partial_cp_d21(self, idx=2):
        ''' Return partial derivative of cp to d21. '''
        # initialize list
        der_cp_d21 = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d1 = self.ref_data.data[1,1]
        d2 = self.ref_data.data[idx,idx]
        d12 = self.los_data.data[1,idx]
        der_veca_d21 = self.partial_veca_d21(idx) # TODO : compute one time

        for j in range(3):
            der_cp_d21.append(
                2 * d1.T @ d12 @ d12.T @ (
                    der_veca_d21[j] @ veca.T + veca @ der_veca_d21[j].T
                ) @ d2
            )

        return der_cp_d21


    def partial_cp_d2(self, idx=2):
        ''' Return partial derivative of cp to d2. '''
       # initialize list
        der_cp_d2 = []

        # parse variables
        veca = self.compute_vec_a(idx)
        d1 = self.ref_data.data[1,1]
        d12 = self.los_data.data[1,idx]

        for j in range(3):
            prn_axs = get_principal_axis(3, j)
            der_cp_d2.append(
                d1.T @ d12 @ d12.T @ rot_mtx(np.pi, veca) @ prn_axs
            )

        return der_cp_d2


    def compute_other_covariance(self):
        '''
            Return covar of ri2 and ri3.
        '''

        for idx_rel in [2,3]:
            cov_ine = np.zeros((3,3))
            for i in range(3):
                for j in range(3):
                    cov_ine[i,j] = self.get_element_other(idx_rel, i, j)
            
            # append to point
            self.cov_pnt.data[(0,idx_rel)] = -cov_ine + np.trace(cov_ine) * np.eye(3)
            # self.cov_pnt.data[(0,idx_rel)] = -(cov_ine + 0.5 * np.trace(cov_ine) * np.eye(3))
            # self.cov_pnt.data[(0,idx_rel)] = cov_ine

            # append sdevs
            self.sdev_pnt.data[(0,idx_rel)] = self.compute_axis_sdev(-cov_ine + np.trace(cov_ine) * np.eye(3))
            # self.sdev_pnt.data[(0,idx_rel)] = self.compute_axis_sdev(-(cov_ine + 0.5 * np.trace(cov_ine) * np.eye(3)))
            # self.sdev_pnt.data[(0,idx_rel)] = self.compute_axis_sdev(cov_ine)

        return 



    def get_element_other(self, idx_rel, i, j):
        # compute sum of covariance
        cov_da = 0
        for k in range(3):
            cov_da += (
                self.atd_data.data[0,1].T[:,i].reshape((3,1)).T @
                self.compute_covar_rel_col(idx_rel, k, k) @
                self.atd_data.data[0,1].T[:,j].reshape((3,1))
            )
            cov_da += (
                self.atd_data.data[1,idx_rel][:,k].reshape((3,1)).T @
                self.compute_covar_rel_ine_col(idx_rel, i, k) @
                self.atd_data.data[0,1].T[:,j].reshape((3,1))
            )
            cov_da += (
                self.atd_data.data[1,idx_rel][:,k].reshape((3,1)).T @
                self.compute_covar_rel_ine_col(idx_rel, k, j) @
                self.atd_data.data[0,1].T[:,i].reshape((3,1))
            )
            cov_da += (
                self.atd_data.data[1,idx_rel][:,k].reshape((3,1)).T @
                self.compute_covar_finaline_col(i, j) @
                self.atd_data.data[1,idx_rel][:,k].reshape((3,1))
            )

        return cov_da



def relative_covariance_lstsqr(R12, d1, d2, d21, rd1, rd2, rd21, rd12):
    '''
    Return relative covariance of a leastsquares problem.

    Reference
    ---------
        Linares and Crassidis, Constrained Relative Attitude Determination for
        Two-Vehicle Formations, 2011

    Arguments
    ---------
    R12 : numpy.ndarray
        Relative rotation
    d1 : numpy.ndarray
        Inertial measurement
    d2 : numpy.ndarray
        Inertial measurement
    d21 : numpy.ndarray
        LOS measurement
    rd1 : numpy.ndarray
        Measurement noise
    rd2 : numpy.ndarray
        Measurement noise
    rd21 : numpy.ndarray
        Measurement noise
    rd12 : numpy.ndarray
        Measurement noise

    Returns
    -------
    list
        Covariance and intermediate covariance values.

    '''
    # covariance of 12
    rDelta1 = rd12 + R12 @ rd21 @ np.transpose(R12)
    rDelta2 = np.transpose(d1) @ R12 @ rd2 @ np.transpose(R12) @ d1 + np.transpose(R12 @ d2) @ rd1 @ R12 @ d2
    rDelta12 = np.zeros((3,1))

    # matrix R
    aux1 = np.concatenate([rDelta1, rDelta12], axis=1)
    aux2 = np.concatenate([np.transpose(rDelta12), rDelta2], axis=1)
    covR12 = np.concatenate([aux1, aux2], axis=0)

    # matrix H
    # TODO test
    aux1 = -1 * skew_sym(R12 @ d21)
    aux2 = -1 * np.transpose(d1) @ skew_sym(R12 @ d2)
    covH12 = np.concatenate([aux1, aux2], axis=0)

    # covariance expression
    # TODO test singular matrix
    P12 = np.linalg.inv( np.transpose(covH12) @ np.linalg.inv(covR12) @ covH12 )

    return P12, covH12, covR12
