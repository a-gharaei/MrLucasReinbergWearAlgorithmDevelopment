import matplotlib.pyplot as plt
import numpy as np
import pygeos
import trimesh
from mpl_toolkits import mplot3d
from scipy.spatial.transform import Rotation

import helpers as hlp
import helpers3D as hlp3
from geometrics import Vector
from grain import Grain3D

mesh = trimesh.load("./Grains/Cuboctahedron.stl")
print(mesh.bounds)
