import os
from operator import pos

import pandas as pd
from matplotlib.colors import LightSource
from scipy import signal
from SimulationToolbox.Geometry.geometric_objects import *
from SimulationToolbox.Geometry.geometry import *
from SimulationToolbox.PhysicalObjects.grain import *
from SimulationToolbox.PhysicalObjects.tool import Tool
from SimulationToolbox.Simulation.kinematics import Kinematics
from SimulationToolbox.Simulation.physical_models import (
    AttritiousWearModelResultList,
    GrainForceModelResultListNumpyArrays,
    ToolForceModelResultListFloat,
)
from SimulationToolbox.Visualization.visualization import *
from SimulationToolbox.Visualization.visualization_objects import *

configuration.do_plot = True

# Paths for different processes
tutorial = r"./SimulationResults/MyFancySimulation/"
disc_cutting = r"./SimulationResults/DiscCutting/"
single_grain_scratch_test = r"./SimulationResults/SingleGrainScratch/"

# choose process to analyse
path = single_grain_scratch_test

wp = Workpiece.load_from_disk(os.path.join(path, "wp_after_grinding.pkl"))
tool = Tool.load_from_disk(os.path.join(path, "tool_after_grinding.pkl"))
kin = Kinematics.load_from_disk(os.path.join(path, "kin.pkl"))
tool_forces = ToolForceModelResultListFloat.load_from_disk(
    os.path.join(path, "tool_forces.pkl")
)
grain_forces_array = GrainForceModelResultListNumpyArrays.load_from_disk(
    os.path.join(path, "grain_forces.pkl")
)
wear_model_results = AttritiousWearModelResultList.load_from_disk(
    os.path.join(path, "wear_model_results.pkl")
)  # Wear magnitudes per grain per process step
with open(os.path.join(path, "fracture_wear_results.pkl"), "rb") as file:
    fracture_wear_model_results = dill.load(file)
with open(os.path.join(path, "fractured_grains.pkl"), "rb") as file:
    fractured_grains_list = dill.load(file)
with open(os.path.join(path, "fracture_data.pkl"), "rb") as file:
    fracture_data = dill.load(file)
# plot and analyze forces

# Calculate tool forces in global frame
tool_forces = tool_forces.get_forces_in_global(kin.ToolTrajectory)
tool_forces_abs = np.sqrt(
    np.square(tool_forces.tool_forces_x)
    + np.square(tool_forces.tool_forces_y)
    + np.square(tool_forces.tool_forces_z)
)

# Calculate magnitude of grain Forces
grain_forces_magnitude = [
    np.sqrt(
        np.square(grain_forces_array.grain_forces_x[grain_idx])
        + np.square(grain_forces_array.grain_forces_y[grain_idx])
        + np.square(grain_forces_array.grain_forces_z[grain_idx])
    )
    for grain_idx in range(len(grain_forces_array.grain_forces_x))
]
# plot fractured grains
if fractured_grains_list:
    grain_1 = fractured_grains_list[0]
    plot_grain(grain_1, GrainPlotConfig.default())

# plot forces
# plot magnitude of tool force
fig, axs = plt.subplots(3, sharex=True)
axs[0].plot(kin.ToolTrajectory.times, tool_forces_abs)
axs[0].set_xlabel("time [s]")
axs[0].set_ylabel("Tool Force [N]")
axs[0].set_title("Tool Force [N]")
axs[0].grid()

# plot components of tool forces
axs[1].plot(kin.ToolTrajectory.times, tool_forces.tool_forces_x, label="F_x")
axs[1].plot(kin.ToolTrajectory.times, tool_forces.tool_forces_y, label="F_y")
axs[1].plot(kin.ToolTrajectory.times, tool_forces.tool_forces_z, label="F_z")
axs[1].set_xlabel("time [s]")
axs[1].set_ylabel("Tool Force [N]")
axs[1].set_title("Tool Force single components [N]")
axs[1].grid()
axs[1].legend()

# plot magnitude of grain forces
for grain_force in grain_forces_magnitude:
    axs[2].plot(kin.ToolTrajectory.times, grain_force)
axs[2].set_xlabel("time [s]")
axs[2].set_ylabel("Grain-Forces [N]")
axs[2].set_title("Grain-Forces [N]")
axs[2].grid()
fig.tight_layout()

# plot cumulative/average wear over time

fig, ax = plt.subplots(2)
cumulative_wear_result = []
cumulative_wear = 0
for magnitudes in wear_model_results.wears_magnitude_list:
    if magnitudes:
        cumulative_wear += sum(magnitudes)
    cumulative_wear_result.append(cumulative_wear)
ax[0].plot(cumulative_wear_result)
ax[0].set_xlabel("Time steps")
ax[0].set_ylabel("Cumulative wear (mm)")
ax[0].set_title("Cumulative grain wear over time")

average_wear_result = []
for magnitudes in wear_model_results.wears_magnitude_list:
    if not magnitudes:
        average_wear_result.append(0)
    else:
        average_wear_result.append(np.mean(magnitudes))
ax[1].plot(average_wear_result)
ax[1].set_xlabel("Time steps")
ax[1].set_ylabel("Average wear (mm)")
ax[1].set_title("Average grain wear over time")
fig.tight_layout()

# pandas df with fracture data
fracture_df = pd.DataFrame(
    fracture_data,
    columns=["Rankine stress", "Penetration Depth", "Cutting Force", "Normal Force"],
)
print(fracture_df)

# plot material removed
fig, ax = plt.subplots()
ax.plot(fracture_wear_model_results[0])
ax.set_xlabel("Time steps")
ax.set_ylabel("volume removed ($mm^3$)")
ax.set_title("Volume removed through fracture wear per time step")
fig.tight_layout()

# plot rankine stress and penetration depth
highest_rankine_stress = max(fracture_wear_model_results[1])
fig, ax = plt.subplots(2)
ax[0].text(80, 2, f"highest rankine stress:{highest_rankine_stress}")
ax[0].plot(fracture_wear_model_results[1])
ax[0].set_ylabel("Rankine stress (N)")
ax[0].set_title("Rankine stress average for each timestep")
ax[1].plot(fracture_wear_model_results[2])
ax[1].set_ylabel("Penetration depth (mm)")
ax[1].set_title("Penetration depth average for each timestep")
fig.tight_layout()


# plot grinded workpiece
plot_workpiece(wp, WorkpiecePlotConfig.default(wp))
z_off = -75.5
tol = 0.01
# topography measure
topography = wp.to_topography(tol, z_off)
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
surf = ax.plot_surface(
    topography.X,
    topography.Y,
    topography.Z,
    linewidth=0,
    edgecolor="none",
    antialiased=True,
    cmap=plt.cm.jet,
)
fig.colorbar(surf, shrink=0.5, aspect=5)
ax.set_title("Workpiece Topography")
ax.set_xlabel("x axis (m)")
ax.set_ylabel("y axis (m)")
ax.set_zlabel("z axis (m)")
# ax.set_xlim3d(0, 10)
# ax.set_ylim3d(0, 10)
# ax.set_zlim3d(0, 10)

# Calculate and plot Sa
topography = wp.to_topography(tol, z_off)
kernel = np.ones((10, 10)) * 1 / (10 * 10)
mean = signal.convolve2d(topography.Z, kernel, boundary="symm", mode="same")
diffs = np.abs(topography.Z - mean)
Sa = signal.convolve2d(diffs, kernel, boundary="symm", mode="same")

# Plot shaded Topography and Sa in comparison
fig, axs = plt.subplots(ncols=1, nrows=2)
ls = LightSource(azdeg=315, altdeg=45)
axs[0].imshow(ls.hillshade(topography.Z, vert_exag=1), cmap="gray")
axs[0].set_title("Workpiece Surface from top")
img = axs[1].imshow(Sa, cmap=plt.cm.jet)
axs[1].set_title("Roughness: Sa [1] with Kernel size 1x1")
bar = plt.colorbar(img)
bar.set_label("Sa[1]")
