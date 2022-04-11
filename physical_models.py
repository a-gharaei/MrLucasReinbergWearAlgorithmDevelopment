import numpy as np

import helpers as hlp
from geometrics import Vector
from grain import Grain3D


def rankine_stress(
    grain: Grain3D, F_C: float, F_N: float, P_DEPTH: float, cutting_direction: Vector
) -> float:
    """Calculates rankine stress of a grain

    Args:
        grain (Grain3D): Instance of Grain3D
        F_C (float): Cutting force
        F_N (float): Normal force
        P_DEPTH (float): Penetration depth
        cutting_direction (Vector): direction in which the grain moves

    Returns:
        float: rankine stress
    """
    A_cut = hlp.get_A_cut(grain, P_DEPTH, cutting_direction)
    A_ortho = hlp.get_A_ortho(grain, P_DEPTH)
    sigma_c = F_C / A_cut
    sigma_n = F_N / A_ortho
    tau = F_C / A_ortho
    return ((sigma_c + sigma_n) + np.sqrt((sigma_c - sigma_n) ** 2 + 4 * tau**2)) / 2
