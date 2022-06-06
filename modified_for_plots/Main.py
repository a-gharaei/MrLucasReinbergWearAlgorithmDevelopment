import time

import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mpl_toolkits import mplot3d

import grain_functions as gf
import helpers as hlp
from geometrics import Vector, mesh_rotation
from grain import Grain3D
from wear_algorithm import macro_fracture

# Input parameters
F_C = 3000  # Cutting Force in Newton
F_N = 1000  # Normal Force in Newton
P_DEPTH = 0.5  # penetration depth
TENSILE_STRENGTH = 300
cutting_direction = Vector(-1, 0, 0)

# todo: cutting area intersection figure

# Make grain instance
mesh = trimesh.load("./Grains/Cuboctahedron.stl")

# Mesh rotation for plots
mesh = mesh_rotation(mesh, np.pi / 4.2, "z")
mesh = mesh_rotation(mesh, np.pi / 2, "y")
mesh = mesh_rotation(mesh, np.pi / 20, "x")
mesh = mesh_rotation(mesh, np.pi / 3.5, "z")


# figure = plt.figure()
# axes = figure.add_subplot(projection="3d")
# axes.add_collection3d(
#     mplot3d.art3d.Poly3DCollection(mesh.triangles, alpha=0.7, edgecolors="k")
# )
# scale = mesh.vertices.flatten()
# axes.auto_scale_xyz(scale, scale, scale)
# axes.set_xlabel("X [$mm$]")
# axes.set_ylabel("Y [$mm$]")
# axes.set_zlabel("Z [$mm$]")
# axes.yaxis.set_ticklabels([])
# plt.show()


grain = Grain3D(mesh)
# hlp.plot_trimesh(grain.mesh)

# time start
initial_time = time.time()
grain.mesh = macro_fracture(
    grain, F_C, F_N, P_DEPTH, cutting_direction, TENSILE_STRENGTH
)
end_time = time.time()
figure = plt.figure()
axes = figure.add_subplot(projection="3d")
axes.add_collection3d(
    mplot3d.art3d.Poly3DCollection(grain.mesh.triangles, alpha=0.7, edgecolors="k")
)
scale = grain.mesh.vertices.flatten()
axes.auto_scale_xyz(scale, scale, scale)
axes.set_xlabel("X [$mm$]")
axes.set_ylabel("Y [$mm$]")
axes.set_zlabel("Z [$mm$]")
axes.xaxis.set_ticklabels([])
plt.show()
print(f"The wear algorithm took {end_time-initial_time} seconds")
# hlp.plot_trimesh(grain.mesh)
