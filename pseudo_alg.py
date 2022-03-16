import random
import numpy as np
import matplotlib.pyplot as plt
import wear_functions as wf
# Pseudo algotithm for fracture wear on a single abrasive grain (triangle)

ex = np.array([1, 0])  # unit-vector in x-direction
ey = np.array([0, 1])  # unit-vector in y-direction

# Input parameters
tensile_strength = 300  # 300 Mpa
F_c = 3 * ex  # Cutting Force in Newton
F_n = 1 * ey  # Normal Force in Newton
P = F_c + F_n  # Combined force vector
d = 1 * ey  # penetration depth

# Grain vertice coordinates
# x-coordinates of abrasive grain vertices
x = np.array([0, -3, 3, 2, 0], dtype=float)
# y-coordinates of abrasive grain vertices
y = np.array([0, 7, 7, 0.5, 0], dtype=float)

# Geometric Calculations
omega = wf.get_angle(P, ey)  # Angle of attack of the force
rake = wf.get_vector(np.array([x[0], y[0]]), np.array(
    [x[1], y[1]]))
alpha = wf.get_angle(rake, ey)
h = rake[1]  # height of trianlge
# vertice of grain with min y value -> max penetration depth -> cutting edge
cutting_edge = wf.get_cutting_edge(x, y)
# Vector for contact lenght
conlen = np.array([-np.linalg.norm(d) * np.tan(alpha), np.linalg.norm(d)])
conlen_value = np.linalg.norm(conlen)  # contact length
# vector from cutting edge to estimated "danger point" with maximum tenisle stress
point_of_danger = random.uniform(2, 3) * conlen

rho = np.pi / 2 - alpha  # Angle between x-axes and rake face
stress = wf.get_radial_stress(
    omega, rho, alpha, np.linalg.norm(P), point_of_danger)  # returns radial stress at danger point (Hardcoded atm (returns 400))

# checks if stress is bigger than tensile strength -> modifies triangle by adding a limited randomized crack
if stress > tensile_strength:
    x_mod, y_mod = wf.make_crack(
        x, y, cutting_edge, rake, point_of_danger)
else:
    x_mod, y_mod = x, y


# Plotting original and modified grain
fig, (ax1, ax2) = plt.subplots(1, 2)
fig.suptitle('Original and modified grain')
ax1.plot(x, y)
ax2.plot(x_mod, y_mod)
plt.show()
