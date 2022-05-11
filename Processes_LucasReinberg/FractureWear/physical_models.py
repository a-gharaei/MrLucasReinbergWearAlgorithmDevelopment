import numpy as np

import FractureWear.helpers as hlp
from FractureWear.geometrics import Vector
from FractureWear.grain import Grain3D


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
    cut_polygon, A_cut = hlp.get_A_cut(grain, P_DEPTH, cutting_direction)
    ortho_polygon, A_ortho = hlp.get_A_ortho(grain, P_DEPTH)
    if A_ortho == 0:
        return 0, 0, 0
    if A_cut == 0:
        return 0, 0, 0
    sigma_c = F_C / A_cut
    sigma_n = F_N / A_ortho
    tau = F_C / A_ortho
    return (
        ((sigma_c + sigma_n) + np.sqrt((sigma_c - sigma_n) ** 2 + 4 * tau**2)) / 2,
        cut_polygon,
        ortho_polygon,
    )
