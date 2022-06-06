import time
from ast import Del
from re import U

import matplotlib as mpl
import meshcut
import numpy as np
import numpy.linalg as la
import trimesh
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d as a3
from scipy.spatial import ConvexHull
from shapely.geometry import LineString, Point, Polygon

import helpers as hlp
import helpers3D as hlp3
import physical_models as pm
from geometrics import Vector
from grain import Grain2D, Grain3D

# Input parameters
F_C = 3000  # Cutting Force in Newton
F_N = 1000  # Normal Force in Newton
P_DEPTH = 0.07  # penetration depth
TENSILE_STRENGTH = 300
cutting_direction = Vector(1, 1, 0)

original_mesh = trimesh.load("./Grains/Cuboctahedron.stl")
grain = Grain3D(original_mesh)
hlp.plot_trimesh(original_mesh)

initial_time = time.time()
# Rankine stress calculation
rankine = pm.rankine_stress_3D(grain, F_C, F_N, P_DEPTH, cutting_direction)

rotated_mesh, rotation_angle = hlp3.rotate_grain(grain, cutting_direction)

projected_area = hlp3.project_grain(grain.mesh)
grain2d = Grain2D(projected_area)
grain2d.initializeValues(F_C, F_N, P_DEPTH)

# Rankine criterion
if rankine > TENSILE_STRENGTH:

    # Crack 2D grain and get plane origin and normal
    grain2d.make_crack(F_C, F_N)
    plane_orig = [grain2d.plane_orig[0], 0, grain2d.plane_orig[1]]
    plane_norm = [grain2d.plane_norm[0], 0, grain2d.plane_norm[1]]

cutting_polygon = hlp3.cross_section(rotated_mesh, plane_norm, plane_orig)

cutting_polygon = cutting_polygon.tolist()

new_vertices = hlp3.get_new_vertices(
    original_mesh.vertices, plane_orig, plane_norm, cutting_polygon
)
vertices = np.array([np.array(x) for x in new_vertices], dtype=object)

hull = ConvexHull(vertices)
indices = hull.simplices

new_mesh_rotated = trimesh.Trimesh(vertices=vertices, faces=indices)

new_mesh = hlp3.mesh_z_rotation(new_mesh_rotated, -rotation_angle)

end_time = time.time()
# Time end
time1 = end_time - initial_time
print(time1)

hlp.plot_trimesh(new_mesh)
