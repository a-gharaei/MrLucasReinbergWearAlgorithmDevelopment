from operator import pos
from SimulationToolbox.Simulation.wear_models import AttritiousWearModelResultList
from SimulationToolbox.Simulation.physical_models import GrainForceModelResultListNumpyArrays, ToolForceModelResultListFloat
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


def tutorial_run():
    # load data from past simulation
    path = "./Tutorials/Resources/SimulationResults/MyFancySimulation"
    wp = Workpiece.load_from_disk(os.path.join(path, 'wp_after_grinding.pkl'))
    tool = Tool.load_from_disk(os.path.join(path, 'tool_after_grinding.pkl'))
    kin = Kinematics.load_from_disk(os.path.join(path, 'kin.pkl'))
    grain_forces = GrainForceModelResultListNumpyArrays.load_from_disk(
        os.path.join(path, 'grain_forces.pkl'))  # Forces in individual grain frames
    tool_forces = ToolForceModelResultListFloat.load_from_disk(
        os.path.join(path, 'tool_forces.pkl'))  # Forces in tool frame at each process step
    global_forces = ToolForceModelResultListFloat.load_from_disk(
        os.path.join(path, 'global_forces.pkl'))  # Forces in global frame of the process
    wear_model_results = AttritiousWearModelResultList.load_from_disk(
        os.path.join(path, 'wear_model_results.pkl'))  # Wear magnitudes per grain per process step

    # plot and analyze forces
    # Calculate magnitude of grain Forces
    grain_forces_magnitude = []
    for grain_idx in range(len(grain_forces.grain_forces_x)):
        grain_forces_magnitude.append(
            np.sqrt(np.square(grain_forces.grain_forces_x[grain_idx])
                    + np.square(grain_forces.grain_forces_y[grain_idx])
                    + np.square(grain_forces.grain_forces_z[grain_idx])))
    # plot forces
    # plot magnitude of global force
    global_forces_abs = np.sqrt(np.square(global_forces.tool_forces_x)
                                + np.square(global_forces.tool_forces_y)
                                + np.square(global_forces.tool_forces_z))

    fig, axs = plt.subplots(2, sharex=True)
    axs[0].plot(kin.ToolTrajectory.times, global_forces_abs)
    axs[0].set_xlabel('time [s]')
    axs[0].set_ylabel('Global Force [N]')
    axs[0].set_title('Global force magnitude [N]')
    axs[0].grid()

    # plot magnitude of grain forces
    for grain_force in grain_forces_magnitude:
        axs[1].plot(kin.ToolTrajectory.times, grain_force)
    axs[1].set_xlabel('time [s]')
    axs[1].set_ylabel('Grain Forces [N]')
    axs[1].set_title('Grain forces magnutide [N]')
    axs[1].grid()
    fig.tight_layout()

   # plot components of tool forces in global frame

    fig, ax = plt.subplots(3, sharex=True)
    ax[0].plot(kin.ToolTrajectory.times,
               global_forces.tool_forces_x, label='F_x')
    ax[0].set_xlabel('time [s]')
    ax[0].set_ylabel('Force - x [N]')
    ax[0].set_title('Global Force x component [N]')
    ax[0].grid()

    ax[1].plot(kin.ToolTrajectory.times,
               global_forces.tool_forces_y, label='F_y')
    ax[1].set_xlabel('time [s]')
    ax[1].set_ylabel('Force - y [N]')
    ax[1].set_title('Global Force y component [N]')
    ax[1].grid()

    ax[2].plot(kin.ToolTrajectory.times,
               global_forces.tool_forces_z, label='F_z')
    ax[2].set_xlabel('time [s]')
    ax[2].set_ylabel('Force - z [N]')
    ax[2].set_title('Global Force z component [N]')
    ax[2].grid()
    fig.tight_layout()

    # plot forces in a new frame
    # pose of new frame with respect to global frame

    new_frame_pose = Pose.from_rotation_axis_angle(Vector.e_z(), np.pi/2)
    # new_frame_pose = kin.ToolTrajectory.poses[0] # initial tool pose as new frame

    # forces in new frame
    new_frame_forces = global_forces.get_global_forces_in_new_frame(
        new_frame_pose)

    fig, ax = plt.subplots(3, sharex=True)
    ax[0].plot(kin.ToolTrajectory.times,
               new_frame_forces.tool_forces_x, label='F_x')
    ax[0].set_xlabel('time [s]')
    ax[0].set_ylabel('Force - x [N]')
    ax[0].set_title('New Frame Force x component [N]')
    ax[0].grid()

    ax[1].plot(kin.ToolTrajectory.times,
               new_frame_forces.tool_forces_y, label='F_y')
    ax[1].set_xlabel('time [s]')
    ax[1].set_ylabel('Force - y [N]')
    ax[1].set_title('New Frame Force y component [N]')
    ax[1].grid()

    ax[2].plot(kin.ToolTrajectory.times,
               new_frame_forces.tool_forces_z, label='F_z')
    ax[2].set_xlabel('time [s]')
    ax[2].set_ylabel('Force - z [N]')
    ax[2].set_title('New Frame Force z component [N]')
    ax[2].grid()
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
    ax[0].set_xlabel('Time steps')
    ax[0].set_ylabel('Cumulative wear (mm)')
    ax[0].set_title('Cumulative grain wear over time')

    average_wear_result = []
    for magnitudes in wear_model_results.wears_magnitude_list:
        if not magnitudes:
            average_wear_result.append(0)
        else:
            average_wear_result.append(np.mean(magnitudes))
    ax[1].plot(average_wear_result)
    ax[1].set_xlabel('Time steps')
    ax[1].set_ylabel('Average wear (mm)')
    ax[1].set_title('Average grain wear over time')
    fig.tight_layout()

    # plot grinded workpiece
    plot_workpiece(wp, WorkpiecePlotConfig.default(wp))

    # topography measure
    topography = wp.to_topography(0.1, 0.5)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(topography.X,
                           topography.Y,
                           topography.Z,
                           linewidth=0,
                           edgecolor='none',
                           antialiased=True,
                           rstride=1, cstride=1,
                           cmap=plt.cm.jet,
                           vmin=4.75, vmax=4.85)
    fig.colorbar(surf, shrink=0.5, aspect=5)
    ax.set_title('Workpiece Topography')
    ax.set_xlabel('x axis (m)')
    ax.set_ylabel('x axis (m)')
    ax.set_zlabel('x axis (m)')
    ax.set_xlim3d(0, 10)
    ax.set_ylim3d(0, 10)
    ax.set_zlim3d(0, 10)

    # Calculate and plot Sa
    topography = wp.to_topography(0.1, 0.5)
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

    if configuration.do_plot:
        plt.show()


if __name__ == "__main__":
    configuration.do_plot = True
    tutorial_run()
