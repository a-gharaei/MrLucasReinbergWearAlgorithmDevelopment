import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d

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
    A_ortho, ortho_plane = hlp.get_A_ortho(grain, P_DEPTH)

    x1, y1, z1 = hlp.polygon_to_plot_3d(ortho_plane)
    figure = plt.figure()
    axes = figure.add_subplot(projection="3d")
    axes.add_collection3d(
        mplot3d.art3d.Poly3DCollection(grain.mesh.triangles, alpha=0.15, edgecolors="k")
    )
    axes.plot_trisurf(x1, y1, z1, alpha=0.9, color="r", edgecolors="k")
    scale = grain.mesh.vertices.flatten()
    axes.auto_scale_xyz(scale, scale, scale)
    axes.set_xlabel("X [$mm$]")
    axes.set_ylabel("Y [$mm$]")
    axes.set_zlabel("Z [$mm$]")
    axes.yaxis.set_ticklabels([])
    plt.show()

    sigma_c = F_C / A_cut
    sigma_n = F_N / A_ortho
    tau = F_C / A_ortho
    return ((sigma_c + sigma_n) + np.sqrt((sigma_c - sigma_n) ** 2 + 4 * tau**2)) / 2
