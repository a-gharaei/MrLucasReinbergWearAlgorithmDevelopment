import numpy as np
import matplotlib.pyplot as plt
import grain
import helpers as hlp
import time


# Pseudo algotithm for fracture wear on a single abrasive grain (triangle)

# Input parameters
tensile_strength = 300  # 300 Mpa
F_c = 300  # Cutting Force in Newton
F_n = 100  # Normal Force in Newton  # Combined force vector
d = 1  # penetration depth


# Set grain vertices
a = np.array([[0, 0], [-1, 3], [-2, 7], [2, 7]], dtype=float)

#Make Grain2D Instance
grain = grain.Grain2D(a)

# set initial values depending on the forces and the penetration depth
grain.initializeValues(F_c, F_n, d)

# Simulation
initial_time = time.time()
grain.make_crack(F_c, F_n)
time1 = time.time()
print(time1-initial_time)

# Plotting
fig, axs = plt.subplots()
fig.suptitle('Original and modified grain')
x1, y1 = hlp.polygon_to_plot(grain.vertices[-1])
axs.plot(x1, y1)

plt.show()
