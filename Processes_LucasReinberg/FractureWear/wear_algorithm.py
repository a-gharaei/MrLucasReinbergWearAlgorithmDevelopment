import mesh_conversion
import trimesh
from numpy import dtype
from SimulationToolbox.Geometry.geometry import *
from SimulationToolbox.PhysicalObjects.grain import *
from SimulationToolbox.PhysicalObjects.tool import *
from SimulationToolbox.Simulation.material_removal import *
from SimulationToolbox.Simulation.physical_models import *
from SimulationToolbox.Simulation.wear_models import *

import FractureWear.geometrics as geom
import FractureWear.grain_functions as gf
import FractureWear.helpers as hlp
from FractureWear.grain import Grain2D, Grain3D
from FractureWear.physical_models import rankine_stress


def macro_fracture(grain, F_C, F_N, P_DEPTH, cutting_dir, tensile_strength):
    # convert from iBrus mesh to trimesh

    original_mesh = grain.get_mesh()
    original_mesh_pose = original_mesh.pose
    mesh = mesh_conversion.from_mesh_to_trimesh(original_mesh)
    if not mesh.is_watertight:
        print("mesh is not watertight")

    cutting_direction = geom.Vector(cutting_dir.x(), cutting_dir.y(), cutting_dir.z())
    grain3d = Grain3D(mesh)
    rotated_mesh, rotation_angle = gf.align_grain(grain3d, cutting_direction)
    grain3d.update_mesh(rotated_mesh)
    rankine, cut_polygon, ortho_polygon = rankine_stress(
        grain3d, F_C, F_N, P_DEPTH, geom.Vector(-1, 0, 0)
    )

    # Check rankine criterion
    if rankine < tensile_strength:
        return 0, rankine
    if P_DEPTH > (grain3d.maxz - grain3d.minz) / 3:
        print(
            f"fracture would have occurd but the penetration depth is too big:{P_DEPTH}"
        )
        print(f"grain height: {grain3d.maxz-grain3d.minz}")
        return 0, rankine
    initial_volume = mesh.volume
    # Align cutting direction with x-axis by rotating the mesh
    # rotated_mesh, rotation_angle = gf.align_grain(grain, cutting_dir)

    # Make Grain2D instance for crack calculations
    grain2d = Grain2D(gf.project_grain_on_xz(rotated_mesh), P_DEPTH)

    # Initialize values that are dependent on the forces and the penetration depth

    # Make crack to get the cutting plane origin and normal
    cut_origin, cut_normal = gf.make_crack(grain2d)
    if isinstance(cut_origin, int):
        print("plane origin or normal is an integer")
        return 0, rankine
    # Todo: random inclination angle

    # Cut the mesh with the calculated plane
    additional_vertices = hlp.cross_section(rotated_mesh, cut_origin, cut_normal)

    new_vertices = gf.get_new_vertices(
        rotated_mesh, additional_vertices, cut_origin, cut_normal
    )

    # Generate mesh from new vertices
    new_mesh = gf.generate_mesh(new_vertices)

    # Rotate mesh back to original position
    fractured_mesh = geom.mesh_rotation(new_mesh, -rotation_angle, "z")
    new_mesh = mesh_conversion.from_trimesh_to_mesh(fractured_mesh, original_mesh_pose)
    grain = grain.update_mesh(new_mesh)
    removed_volume = initial_volume - fractured_mesh.volume
    return removed_volume, rankine
