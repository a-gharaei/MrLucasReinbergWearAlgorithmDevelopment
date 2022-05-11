import time

import numpy as np
import trimesh

import geometrics as geom
import helpers as hlp
from geometrics import Vector
from grain import Grain3D
from wear_algorithm import macro_fracture

# Input parameters
F_C = 3000  # Cutting Force in Newton
F_N = 1000  # Normal Force in Newton
P_DEPTH = 0.07  # penetration depth
TENSILE_STRENGTH = 300
cutting_direction = Vector(1, 0, 0)

# Make grain instance
mesh = trimesh.load("./Grains/Cuboctahedron.stl")
grain = Grain3D(mesh)
print(grain.vertices)

# time start
initial_time = time.time()
grain.mesh = macro_fracture(
    grain, F_C, F_N, P_DEPTH, cutting_direction, TENSILE_STRENGTH
)
end_time = time.time()
print(f"The wear algorithm took {end_time-initial_time} seconds")
hlp.plot_trimesh(grain.mesh)
