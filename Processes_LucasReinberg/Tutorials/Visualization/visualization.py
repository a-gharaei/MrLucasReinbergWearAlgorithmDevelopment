"""This scripts serves as a tutorial for users to get familiar with iBrus.
It will showcase how to define various objects needed and how to use plot
configurations to control various properties(color, transparency, linestyle etc.)
of these objects."""


from SimulationToolbox.Visualization.visualization_objects import *
from SimulationToolbox.Visualization.visualization import *
from SimulationToolbox.PhysicalObjects.workpiece import *
from SimulationToolbox.PhysicalObjects.grain import *
from SimulationToolbox.PhysicalObjects.tool import *
from SimulationToolbox.Geometry.geometry import *
from SimulationToolbox.Geometry.geometric_objects import *
from SimulationToolbox.Simulation.kinematics import *
from SimulationToolbox.Simulation.process import *
import random
from typing import *
import matplotlib.pyplot as plt


def tutorial_run():
    # Define object to be plotted
    path = "./Tutorials/Resources/Grains/Cuboctahedron.stl"
    grain = Grain.from_stl(path, Pose.identity(), Pose.identity())

    # plot_{object} with default config
    plot_grain(grain, GrainPlotConfig.default())

    # plot_{object} with custom title and labels
    plot_grain(grain, GrainPlotConfig.default(), title='Custom title',
               axis_labels=['axis_1', 'axis_2', 'axis_3'], length_unit='[mm]')

    # Change plot library
    plot_grain(grain, GrainPlotConfig.default(), PlotLibrary.plotly)

    # Select color of grain faces with classmethod
    plot_config = GrainPlotConfig.from_color(Color.red())
    plot_grain(grain, plot_config)

    # Change attributes of configuration
    plot_config = GrainPlotConfig.from_color(Color.green())
    plot_config.pose_plot_config = PosePlotConfig.hidden()
    plot_grain(grain, plot_config)

    # Use __init__ to specify all properties
    line_plot_config = LinePlotConfig(color=Color.black(),
                                      width=1,
                                      style=Linestyle.dashed,
                                      alpha=1)

    poly_plot_config = PolyPlotConfig(face_color=Color.red(),
                                      boundary=line_plot_config,
                                      alpha=0.3)

    plot_config = GrainPlotConfig(MeshPlotConfig(
        pose_plot_config=PosePlotConfig.default(),
        surface_config=poly_plot_config))

    plot_grain(grain, plot_config)

    # Plot configs for objects with subobjects (eg. Workpiece = List of Flatmanifolds)

    # Plot workpiece based on a user defined and dynamic config
    box = Box(0, 100, 0, 50, 0, 50, Pose.identity())
    workpiece = Workpiece.from_box(box, 10)

    def get_flatmanifold_config_random_color(flatmanifold: FlatManifold) \
            -> FlatManifoldPlotConfig:
        color_list = [Color.red(), Color.black(), Color.blue(),
                      Color.green(), Color.magenta()]
        return FlatManifoldPlotConfig.from_color(random.choice(color_list),
                                                 random.choice(color_list))

    plot_workpiece(workpiece, WorkpiecePlotConfig.from_workpiece_and_callable(
        workpiece, get_flatmanifold_config_random_color))

    if configuration.do_plot:
        plt.show()


if __name__ == "__main__":
    configuration.do_plot = False
    tutorial_run()
