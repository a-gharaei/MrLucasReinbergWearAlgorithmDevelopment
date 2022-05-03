from operator import pos
from SimulationToolbox.Simulation.physical_models import GrainForceModelResultListNumpyArrays, ToolForceModelResultListFloat, AttritiousWearModelResultList
from SimulationToolbox.PhysicalObjects.tool import Tool
from SimulationToolbox.PhysicalObjects.grain import *
from SimulationToolbox.Geometry.geometric_objects import *
from SimulationToolbox.Geometry.geometry import *
from SimulationToolbox.Visualization.visualization_objects import *
from SimulationToolbox.Visualization.visualization import *
from SimulationToolbox.Simulation.kinematics import Kinematics
import os
from scipy import signal
from matplotlib.colors import LightSource

configuration.do_plot = True

# load data from past simulation
path = r"./SimulationResults/"
wp = Workpiece.load_from_disk(os.path.join(path, 'wp_after_grinding.pkl'))
tool = Tool.load_from_disk(os.path.join(path, 'tool_after_grinding.pkl'))
kin = Kinematics.load_from_disk(os.path.join(path, 'kin.pkl'))
tool_forces = ToolForceModelResultListFloat.load_from_disk(
    os.path.join(path, 'tool_forces.pkl'))
grain_forces_array = GrainForceModelResultListNumpyArrays.load_from_disk(
    os.path.join(path, 'grain_forces.pkl'))
# wear_model_results = AttritiousWearModelResultList.load_from_disk(
#     os.path.join(path, 'wear_model_results.pkl'))  # Wear magnitudes per grain per process step

# plot and analyze forces

# Calculate tool forces in global frame
tool_forces = tool_forces.get_forces_in_global(kin.ToolTrajectory)
tool_forces_abs = np.sqrt(np.square(tool_forces.tool_forces_x)
                          + np.square(tool_forces.tool_forces_y)
                          + np.square(tool_forces.tool_forces_z))

# Calculate magnitude of grain Forces
grain_forces_magnitude = []
for grain_idx in range(len(grain_forces_array.grain_forces_x)):
    grain_forces_magnitude.append(
        np.sqrt(np.square(grain_forces_array.grain_forces_x[grain_idx])
                + np.square(grain_forces_array.grain_forces_y[grain_idx])
                + np.square(grain_forces_array.grain_forces_z[grain_idx])))
# plot forces
# plot magnitude of tool force
fig, axs = plt.subplots(3, sharex=True)
axs[0].plot(kin.ToolTrajectory.times, tool_forces_abs)
axs[0].set_xlabel('time [s]')
axs[0].set_ylabel('Tool Force [N]')
axs[0].set_title('Tool Force [N]')
axs[0].grid()

# plot components of tool forces
axs[1].plot(kin.ToolTrajectory.times,
            tool_forces.tool_forces_x, label='F_x')
axs[1].plot(kin.ToolTrajectory.times,
            tool_forces.tool_forces_y, label='F_y')
axs[1].plot(kin.ToolTrajectory.times,
            tool_forces.tool_forces_z, label='F_z')
axs[1].set_xlabel('time [s]')
axs[1].set_ylabel('Tool Force [N]')
axs[1].set_title('Tool Force single components [N]')
axs[1].grid()
axs[1].legend()

# plot magnitude of grain forces
for grain_force in grain_forces_magnitude:
    axs[2].plot(kin.ToolTrajectory.times, grain_force)
axs[2].set_xlabel('time [s]')
axs[2].set_ylabel('Grain-Forces [N]')
axs[2].set_title('Grain-Forces [N]')
axs[2].grid()
fig.tight_layout()

# plot cumulative/average wear over time

# fig, ax = plt.subplots(2)
# cumulative_wear_result = []
# cumulative_wear = 0
# for magnitudes in wear_model_results.wears_magnitude_list:
#     if magnitudes:
#         cumulative_wear += sum(magnitudes)
#     cumulative_wear_result.append(cumulative_wear)
# ax[0].plot(cumulative_wear_result)
# ax[0].set_xlabel('Time steps')
# ax[0].set_ylabel('Cumulative wear (mm)')
# ax[0].set_title('Cumulative grain wear over time')

# average_wear_result = []
# for magnitudes in wear_model_results.wears_magnitude_list:
#     if not magnitudes:
#         average_wear_result.append(0)
#     else:
#         average_wear_result.append(np.mean(magnitudes))
# ax[1].plot(average_wear_result)
# ax[1].set_xlabel('Time steps')
# ax[1].set_ylabel('Average wear (mm)')
# ax[1].set_title('Average grain wear over time')
# fig.tight_layout()

# plot grinded workpiece
plot_workpiece(wp, WorkpiecePlotConfig.default(wp))
z_off = -75.5
tol = 0.01
# topography measure
topography = wp.to_topography(tol, z_off)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(topography.X,
                       topography.Y,
                       topography.Z,
                       linewidth=0,
                       edgecolor='none',
                       antialiased=True,
                       cmap=plt.cm.jet,
                       )
fig.colorbar(surf, shrink=0.5, aspect=5)
ax.set_title('Workpiece Topography')
ax.set_xlabel('x axis (m)')
ax.set_ylabel('y axis (m)')
ax.set_zlabel('z axis (m)')
# ax.set_xlim3d(0, 10)
# ax.set_ylim3d(0, 10)
# ax.set_zlim3d(0, 10)

# Calculate and plot Sa
topography = wp.to_topography(tol, z_off)
kernel = np.ones((10, 10))*1/(10*10)
mean = signal.convolve2d(topography.Z,
                         kernel,
                         boundary='symm',
                         mode='same')
diffs = np.abs(topography.Z-mean)
Sa = signal.convolve2d(diffs,
                       kernel,
                       boundary='symm',
                       mode='same')

# Plot shaded Topography and Sa in comparison
fig, axs = plt.subplots(ncols=1, nrows=2)
ls = LightSource(azdeg=315, altdeg=45)
axs[0].imshow(ls.hillshade(topography.Z, vert_exag=1), cmap='gray')
axs[0].set_title('Workpiece Surface from top')
img = axs[1].imshow(Sa, cmap=plt.cm.jet)
axs[1].set_title('Roughness: Sa [1] with Kernel size 1x1')
bar = plt.colorbar(img)
bar.set_label('Sa[1]')
