import numpy as np
import matplotlib.pyplot as plt
import grain
import helpers as hlp


# Pseudo algotithm for fracture wear on a single abrasive grain (triangle)

# Input parameters
tensile_strength = 300  # 300 Mpa
F_c = 300  # Cutting Force in Newton
F_n = 100  # Normal Force in Newton  # Combined force vector
d = 1  # penetration depth


# Set grain vertices
a = np.array([[0, 0], [-2, 7], [2, 7]], dtype=float)

#Make Grain2D Instance
grain = grain.Grain2D(a)

# set initial values depending on the forces and the penetration depth
grain.initializeValues(F_c, F_n, d)


# Simulation
for i in range(10):
    grain.make_crack(F_c, F_n)
    if grain.finish:
        break


# Plotting
fig, axs = plt.subplots(2, 2)
fig.suptitle('Original and modified grain')
x1, y1 = hlp.polygon_to_plot(grain.vertices[1])
x2, y2 = hlp.polygon_to_plot(grain.vertices[2])
x3, y3 = hlp.polygon_to_plot(grain.vertices[-2])
x4, y4 = hlp.polygon_to_plot(grain.vertices[-1])
axs[0, 0].plot(x1, y1)
axs[0, 0].set_title('Original grain')
axs[0, 1].plot(x2, y2)
axs[0, 1].set_title('Grain after first simulation step')
axs[1, 0].plot(x3, y3)
axs[1, 0].set_title('Grain after second simulation step')
axs[1, 1].plot(x4, y4)
axs[1, 1].set_title('Grain after third simulation step')
plt.show()
