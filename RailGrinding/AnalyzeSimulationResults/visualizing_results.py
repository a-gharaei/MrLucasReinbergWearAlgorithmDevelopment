from operator import pos
import os

import matplotlib.pyplot as plt
from scipy import signal
from matplotlib.colors import LightSource
import pandas as pd
import matplotlib.pyplot as plt

from SimulationToolbox.Simulation.physical_models import GrainForceModelResultListNumpyArrays
from SimulationToolbox.Simulation.physical_models import ToolForceModelResultListFloat
from SimulationToolbox.Visualization.visualization import *
from SimulationToolbox.Simulation.kinematics import Kinematics


# df = pd.read_csv(r"./../MultipleRunSimulations/Results/Exp0/Report.csv")

def save_all_labels_in_TIFF_format(df_original, dir, time_step_size):
    # df.loc[(df != 0).any(1)]
    # df.drop(df.columns[df.columns.str.contains('unnamed', case=False)], axis=1, inplace=True)

    print(df_original.columns)
    df = df_original.rename({"tool force X": "Tool force X component - N",
                             "tool force Y": 'F$_{c}$ [N]',
                             "tool force Z": 'F$_{n}$ [N]',
                             "No. of grains subject to attritious wear": "Anzahl der Körner",
                             "Cumulative average of magnitude of grains' attritious 3D wear vectors - micrometer": "[µm]",
                             "Sum of projected cutting areas - micrometer^2": "[µm$^{2}$]"}, axis='columns')

    # df = df.loc[1038: 1088]
    df = df.loc[1038: 1088]

    simulation_steps = [i for i in range(df.shape[0])]
    # time = list(np.squeeze(time_step_size* np.array([range(0, df.shape[0])])))

    df['Simulation steps'] = simulation_steps
    df['Time - mSec'] = 1000 * time_step_size * df['Simulation steps']  # mSec

    for i in range(len(df.columns)):
        # importing the required module
        # x axis values
        x = df['Time - mSec']
        # corresponding y axis values
        y = df[df.columns[i]]

        # plotting the points
        plt.plot(x, y)
        plt.rcParams.update({'font.size': 17})
        plt.rcParams["axes.labelweight"] = "bold"
        plt.rcParams.update({'mathtext.default': 'regular'})

        # naming the x axis
        plt.xlabel('Simulationszeit [ms]', fontweight='bold', fontsize=18)
        # naming the y axis

        plt.ylabel(df.columns[i], fontweight='bold', wrap=True, fontsize=18)
        # giving a title to my graph
        # plt.title(df.columns[i], wrap=True)
        plt.tight_layout()

        # function to show the plot
        # plt.show()
        plt.savefig(dir + './' + df.columns[i] + '.tiff', dpi=600)
        plt.close()


# to present results
def present(result_df, path, config):
    # load data from past simulation
    wp = Workpiece.load_from_disk(os.path.join(path, 'first_state_workpiece.pkl'))
    tool = Tool.load_from_disk(os.path.join(path, 'last_state_tool.pkl'))
    kin = Kinematics.load_from_disk(os.path.join(path, 'last_state_kinematics.pkl'))

    # Todo: # visualize Kinematics
    # plot_kinematics(kin, KinematicsPlotConfig.default(),
    #                 library=PlotLibrary.plotly)

    if config['visualize_process']:
        plot_kinematics(kin, KinematicsPlotConfig.default(),
                        library=PlotLibrary.plotly,
                        length_unit='[mm]')

        # ax.azim = -80
        # ax.dist = 10
        # ax.elev = 20
        # plt.savefig(r'C:/Users/spadmin/PycharmProjects/iBrusProcesses/RailGrinding/AnalyzeSimulationResults/image.tiff',
        #             dpi=600)
        # plt.savefig(r'C:/Users/spadmin/PycharmProjects/iBrusProcesses/RailGrinding/AnalyzeSimulationResults/image.jpg',
        #             dpi=600)
        # plt.close()

        # visualize process
        process = Process(tool, wp, kin)
        fm_plot_config = FlatManifoldPlotConfig(PosePlotConfig.default(),
                                                PolyPlotConfig(Color.blue(), LinePlotConfig.default(), 0.1))
        plot_process(process, ProcessPlotConfig(StatePlotConfig(
            ToolPlotConfig.default(tool), WorkpiecePlotConfig([fm_plot_config] * len(wp.slices))),
            KinematicsPlotConfig.default()), kinematics_idx=0, title='', axis_labels=['X', 'Y', 'Z'],
                     length_unit=' [mm]')
        # plot_process(process, ProcessPlotConfig.default(process), length_unit='mm')

        # visualize grain trajectory
        # plot_grain_trajectory(process=process,
        #                       grain_index=0,
        #                       plot_config=ProcessPlotConfig.default(process),
        #                       length_unit='mm')

    if config['visualize_forces']:
        # interactive plot
        if config['visualize_forces_interactive']:
            # pandas_bokeh.output_file('Interactive Plot.html')
            result_df[["tool force X", "tool force Y", "tool force Z"]].plot_bokeh.line()

        # ordinary plot
        tool_forces = ToolForceModelResultListFloat.load_from_disk(
            os.path.join(path, 'simulation_tool_forces.pkl'))
        grain_forces_array = GrainForceModelResultListNumpyArrays.load_from_disk(
            os.path.join(path, 'simulation_grain_forces.pkl'))

        # plot and analyze forces
        # Calculate tool forces in global frame
        tool_forces = tool_forces.get_forces_in_global(kin.ToolTrajectory.poses)
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

    if config['visualize_workpiece']:
        # plot grinded workpiece
        plot_workpiece(wp, WorkpiecePlotConfig.default(wp), axis_labels=['X axis', 'Y axis', 'Z axis'],
                       length_unit='mm')

    if config['visualize_topography']:
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
        ax.set_title('Workpieces Topography')
        ax.set_xlabel('x axis (m)')
        ax.set_ylabel('x axis (m)')
        ax.set_zlabel('x axis (m)')
        ax.set_xlim3d(0, 10)
        ax.set_ylim3d(0, 10)
        ax.set_zlim3d(0, 10)

        # Calculate and plot Sa
        topography = wp.to_topography(0.1, 0.5)
        kernel = np.ones((10, 10)) * 1 / (10 * 10)
        mean = signal.convolve2d(topography.Z,
                                 kernel,
                                 boundary='symm',
                                 mode='same')
        diffs = np.abs(topography.Z - mean)
        Sa = signal.convolve2d(diffs,
                               kernel,
                               boundary='symm',
                               mode='same')

        # Plot shaded Topography and Sa in comparison
        fig, axs = plt.subplots(ncols=1, nrows=2)
        ls = LightSource(azdeg=315, altdeg=45)
        axs[0].imshow(ls.hillshade(topography.Z, vert_exag=1), cmap='gray')
        axs[0].set_title('Workpieces Surface from top')
        img = axs[1].imshow(Sa, cmap=plt.cm.jet)
        axs[1].set_title('Roughness: Sa [1] with Kernel size 1x1')
        bar = plt.colorbar(img)
        bar.set_label('Sa[1]')

        if configuration.do_plot:
            plt.show()


if __name__ == "__main__":
    configuration.do_plot = True
    present()
