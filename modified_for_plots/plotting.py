import numpy as np
import trimesh
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d

import geometrics as geom
import grain_functions as gf
import helpers as hlp
from grain import Grain2D, Grain3D

# simpe polygon plot
x1, y1 = hlp.polygon_to_plot(proj_2d)
fig, ax = plt.subplots()
ax.set_xlabel("X [$mm$]")
ax.set_ylabel("Z [$mm$]")
plt.plot(x1, y1)
ax.set_aspect("equal")
plt.tight_layout()
plt.show()
