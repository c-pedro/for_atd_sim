''' Classes used in the algorithm.
'''
from for_atd_sim.lib_math.so3 import skew_sym

class CoefficientArray:
    '''
    Structure for trigonometric coefficients.

    Instance Variables
    ------------------
    cs
        Sine coefficient.
    cc
        Cosine coefficient.
    cp
        Proportion coefficient.

    '''
    def __init__(self, d1, i1, d2, i2, d12, R):
        self.cs = d1.T @ skew_sym(-d12) @ R @ d2
        self.cc = d1.T @ skew_sym(d12) @ skew_sym(d12) @ R @ d2
        self.cp = (d1.T @ d12 @ d12.T @ R @ d2) - (i1.T @ i2)


class RelativeSolution():
    rot = None

    def __init__(self) -> None:
        self.aux = self.RelativeParams()
        
    class RelativeParams(dict):
        rot_1 = None
        rot_2 = None
        coef = None
        beta = None
        sign = None


class RelativeCandidate():
    def __init__(self) -> None:        
        self.can_a = RelativeSolution()
        self.can_b = RelativeSolution()


class InertiaslSolution():
    rot_i1 = RelativeSolution()
    rot_i2 = RelativeSolution()
    rot_i3 = RelativeSolution()
    rot_12 = RelativeSolution()
    rot_13 = RelativeSolution()        
    rot_23 = RelativeSolution()
    
    rot_x = RelativeSolution()
    rot_y = RelativeSolution()

    def __init__(self) -> None:        
        self.comparison_scores = {}
        # 'optS' : optS,
