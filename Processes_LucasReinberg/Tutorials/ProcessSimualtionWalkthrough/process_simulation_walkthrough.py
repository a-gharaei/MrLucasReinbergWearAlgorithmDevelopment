"""This scripts serves as a tutorial for users to get familiar with iBrus.
It will show case how to define various objects needed and how to define
and run a grinding process."""
import os
import matplotlib.pyplot as plt
from SimulationToolbox.Simulation.process import *
from SimulationToolbox.Utilities.SimulationToolboxConfiguration import configuration
from TemplateProcess.RailGrindingExample.physical_models import *
from TemplateProcess.RailGrindingExample.wear_models import *
from SimulationToolbox.Visualization.visualization import *
from SimulationToolbox.Simulation.material_removal import *
from SimulationToolbox.PhysicalObjects.tool import *
from SimulationToolbox.PhysicalObjects.grain import *
from SimulationToolbox.PhysicalObjects import grain
from SimulationToolbox.Visualization.visualization_objects import GrainPlotConfig, ToolPlotConfig
from SimulationToolbox.Simulation.kinematics import Kinematics
from SimulationToolbox.Visualization.animation import animate_process


def tutorial_run(use_reduced_grain):
    # 1. Define the tool and grain
    # Define the grain shape you want
    path = r"./Tutorials/Resources/Grains/Cuboctahedron.stl"
    grain = Grain.from_stl(path, Pose.identity(), Pose.identity())
    # Generate tool that is using this grain shape
    tool = Tool.from_radius(basic_grain=grain,
                            inner_radius=6,
                            outer_radius=9,
                            number_of_grains=20,
                            tool_pose=Pose.identity())
    # number_of_grains=400 for simulation, here it is 2 for buildserver

    # plot the tool and grain
    grain_plot_config = GrainPlotConfig.default()
    tool_plot_config = ToolPlotConfig.default(tool)
    plot_grain(grain, grain_plot_config)
    plot_tool(tool, tool_plot_config)

    # 2. Define workpiece with 5 planes
    box = Box.from_sizes(size_x=10, size_y=5,
                         size_z=5, pose=Pose.identity())
    # spatial_resolution=0.5 for simulation, here it is 1 for buildserver
    wp = Workpiece.from_box(box, spatial_resolution=0.5)
    plot_workpiece(wp, WorkpiecePlotConfig.default(wp))

    # 1.3 Define kinematics
    feed_rate = 0.2
    start_point = Vector(-9, 2.5, 5.2)
    rotation_axis = Vector.e_z()
    rotational_speed = 30 * (np.pi/180)
    feed = Vector(feed_rate, 0, 0)
    end_time = 45
    time_step_size = 1

    kin = Kinematics.from_feedrate_and_rotation_speed(start_point, rotation_axis,
                                                      rotational_speed, feed,
                                                      end_time, time_step_size)
    # visualize Kinematics
    plot_kinematics(kin, KinematicsPlotConfig.default())
    # visualize process
    process = Process(tool, wp, kin)
    plot_process(process, ProcessPlotConfig.default(process))
    # visualize grain trajectory
    plot_grain_trajectory(process=process,
                          grain_index=0,
                          plot_config=ProcessPlotConfig.default(process))

    # Setting up Simulation
    mat_remover_config = MaterialRemovalConfig(
        use_reduced_grain, True, True, CollisionDetectionConfig.for_partial_tool_wp_contact_process(tool, wp, kin))

    # maximum_distance_travelled_by_grain = 2.5
    start_tool_pose = kin.ToolTrajectory.poses[0]

    matRemover = MaterialRemover(wp,
                                 tool,
                                 start_tool_pose,
                                 kin, mat_remover_config)
    force_model_config = GrainForceModelConfiguration(1, 0.3)
    grain_force_results = []
    tool_force_results = []
    wear_model_results = AttritiousWearModelResultList([], [])

    # Initiate lists for animation input collection
    # workpieces: List[Workpiece] = []
    # tools: List[Tool] = []

    # # Simulation loop
    for current_tool_pose in kin.ToolTrajectory.poses:
        # remove material
        mat_remover_result = \
            matRemover.update(wp,
                              tool,
                              current_tool_pose,
                              mat_remover_config)

        # apply wear
        wear_model_result = apply_attritious_wear_model(
            tool, wear_magnitude_as_penetration_depth_percentage, mat_remover_result)
        apply_attritious_wear(wear_model_result, tool,
                              mat_remover_result, use_reduced_grain)

        # Collecting wp/tool instances for animation
        # workpieces.append(copy.deepcopy(wp))
        # tools.append(copy.deepcopy(tool))

        # apply physical models
        grain_force_model_result = grain_force_model(mat_remover_result,
                                                     tool,
                                                     force_model_config)
        tool_force_model_result = tool_force_model(grain_force_model_result,
                                                   tool)

        # apply bond_wear and pullout
        bond_wear_model_result = apply_archard_bonding_wear_model(tool,
                                                                  1, 1, grain_force_model_result)
        apply_bonding_wear(bond_wear_model_result, tool)
        apply_pullout_condition(
            tool, grain_force_model_result, mat_remover_result, bonding_strength=0.05)

        # capture results in a list
        tool_force_results.append(tool_force_model_result)
        grain_force_results.append(grain_force_model_result)
        wear_model_results.wears_vector_list.append(
            wear_model_result.wears_vector)
        wear_model_results.wears_magnitude_list.append(
            wear_model_result.wears_magnitude)

    # save simulation objects and results to disk for later analization
    # convert to handier format
    grain_forces_array = GrainForceModelResultListNumpyArrays.from_grain_force_model_result_list(
        grain_force_results)  # Forces in grain_frame
    tool_forces_array = ToolForceModelResultListFloat.from_tool_force_model_result_list(
        tool_force_results)  # Forces in tool frame
    global_forces_array = tool_forces_array.get_forces_in_global(
        kin.ToolTrajectory)  # Forces in global frame

    # save to disk
    path = r"./Tutorials/Resources/SimulationResults/MyFancySimulation"
    wp.save_to_disk(os.path.join(path, 'wp_after_grinding.pkl'))
    tool.save_to_disk(os.path.join(path, 'tool_after_grinding.pkl'))
    kin.save_to_disk(os.path.join(path, 'kin.pkl'))
    grain_forces_array.save_to_disk(os.path.join(path, 'grain_forces.pkl'))
    tool_forces_array.save_to_disk(os.path.join(path, 'tool_forces.pkl'))
    global_forces_array.save_to_disk(os.path.join(path, 'global_forces.pkl'))
    wear_model_results.save_to_disk(
        os.path.join(path, 'wear_model_results.pkl'))

    # Function for animation
    # animation_path = r"./Resources/SimulationResults/MyFancySimulation/My_simulation.mp4"
    # animate_process(tools, workpieces, kin, save_path=animation_path)

    if configuration.do_plot:
        plt.show()


if __name__ == "__main__":
    configuration.do_plot = False
    tutorial_run()
