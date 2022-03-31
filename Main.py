import time

import matplotlib.pyplot as plt
import numpy as np

import grain
import helpers as hlp

# Pseudo algotithm for fracture wear on a single abrasive grain (triangle)

# Input parameters
F_C = 3000  # Cutting Force in Newton
F_N = 1000  # Normal Force in Newton
P_DEPTH = 0.5  # penetration depth


# Set grain vertices

b = np.array(
    [[-1, 0], [-3, 2], [-3, 4], [-1, 6], [1, 6], [3, 4], [3, 2], [1, 0]], dtype=float
)

# Make Grain2D Instance
grain = grain.Grain2D(b)
# set initial values depending on the forces and the penetration depth
grain.initializeValues(F_C, F_N, P_DEPTH)

# Simulation
initial_time = time.time()
grain.make_crack(F_C, F_N)
end_time = time.time()
time1 = end_time - initial_time
print(f"It took {grain.fractures} steps and {time1} seconds to crack through")

# Plotting
fig, (ax1, ax2) = plt.subplots(1, 2)
x1, y1 = hlp.polygon_to_plot(grain.vertices[0])
x2, y2 = hlp.polygon_to_plot(grain.vertices[-1])
ax1.plot(x1, y1)
ax1.set_aspect("equal", "box")
ax1.set_title("Unmodified abrasive grain", fontsize=10)
ax2.plot(x2, y2)
ax2.set_aspect("equal", "box")
ax2.set_title("Abrasive grain after σ_critical < σ_result", fontsize=10)

plt.show()
