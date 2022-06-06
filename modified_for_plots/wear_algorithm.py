import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d

import geometrics as geom
import grain_functions as gf
import helpers as hlp
from grain import Grain2D
from physical_models import rankine_stress


def macro_fracture(grain, F_C, F_N, P_DEPTH, cutting_dir, tensile_strength):
    rankine = rankine_stress(grain, F_C, F_N, P_DEPTH, cutting_dir)
    # Check rankine criterion
    if rankine < tensile_strength:
        return grain

    # Align cutting direction with x-axis by rotating the mesh
    rotated_mesh, rotation_angle = gf.align_grain(grain, cutting_dir)

    # Make Grain2D instance for crack calculations
    grain2d = Grain2D(gf.project_grain_on_xz(rotated_mesh), P_DEPTH)

    # Initialize values that are dependent on the forces and the penetration depth

    # Make crack to get the cutting plane origin and normal
    cut_origin, cut_normal = gf.make_crack(grain2d)

    # Cut the mesh with the calculated plane
    additional_vertices = hlp.cross_section(rotated_mesh, cut_normal, cut_origin)

    new_vertices, plane_coefficients = gf.get_new_vertices(
        rotated_mesh, additional_vertices, cut_origin, cut_normal
    )

    x1, y1, z1 = hlp.polygon_to_plot_3d(new_vertices)
    Xs, Ys, Zs = hlp.polygon_to_plot_3d(additional_vertices)

    x_plane = y_plane = np.linspace(-0.7, 0.7, 10)

    xx, yy = np.meshgrid(x_plane, y_plane)
    z = (
        (-cut_normal[0] * xx - cut_normal[1] * yy + plane_coefficients[3])
        * 1.0
        / cut_normal[2]
    )
    figure = plt.figure()
    axes = figure.add_subplot(projection="3d")
    axes.add_collection3d(
        mplot3d.art3d.Poly3DCollection(
            rotated_mesh.triangles, alpha=0.2, edgecolors="k"
        )
    )
    # axes.plot_surface(xx, yy, z, alpha=0.5, color="r")
    axes.plot_trisurf(Xs, Ys, Zs, alpha=0.5, color="r")
    axes.scatter(Xs, Ys, Zs, color="m", marker="o", alpha=1)
    axes.scatter(x1, y1, z1, color="r", marker=".")
    scale = rotated_mesh.vertices.flatten()
    axes.auto_scale_xyz(scale, scale, scale)
    axes.set_xlabel("X [$mm$]")
    axes.set_ylabel("Y [$mm$]")
    axes.set_zlabel("Z [$mm$]")
    axes.yaxis.set_ticklabels([])
    plt.show()

    # Generate mesh from new vertices
    new_mesh = gf.generate_mesh(new_vertices)

    # Rotate mesh back to original position
    new_mesh = geom.mesh_rotation(new_mesh, -rotation_angle, "z")
    return new_mesh
