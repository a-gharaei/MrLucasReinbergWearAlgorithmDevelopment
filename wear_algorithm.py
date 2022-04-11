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

    new_vertices = gf.get_new_vertices(
        rotated_mesh, additional_vertices, cut_origin, cut_normal
    )

    # Generate mesh from new vertices
    new_mesh = gf.generate_mesh(new_vertices)

    # Rotate mesh back to original position
    new_mesh = geom.mesh_rotation(new_mesh, -rotation_angle, "z")
    return new_mesh
