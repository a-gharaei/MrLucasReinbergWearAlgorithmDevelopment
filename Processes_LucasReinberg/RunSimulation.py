import os
import time
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from SimulationToolbox.Simulation.process import *
from SimulationToolbox.Utilities.SimulationToolboxConfiguration import configuration
from Processes_LucasReinberg.physical_models import *
from Processes_LucasReinberg.wear_models import *
from SimulationToolbox.Visualization.visualization import *
from SimulationToolbox.Simulation.material_removal import *
from SimulationToolbox.PhysicalObjects.tool import *
from SimulationToolbox.PhysicalObjects.grain import *
from SimulationToolbox.PhysicalObjects import grain
from SimulationToolbox.Visualization.visualization_objects import GrainPlotConfig, ToolPlotConfig
from SimulationToolbox.Simulation.kinematics import Kinematics
from SimulationToolbox.Visualization.animation import animate_process


# ------------------------ Start of Simulation --------------------------------
# Initiate simulation related variables
program_time_0 = time.time()
configuration.do_plot = True
animate = False


# -------------------------- Tool creation ------------------------------------
# Define the grain shape you want (import or create)
path = r'./Resources/Cuboctahedron.stl'
grain = Grain.from_stl(path, Pose.identity(), Pose.identity()).scale(0.2)

# Generate tool that is using this grain shape
tool_creation_time0 = time.time()
tool = Tool.from_radius(basic_grain=grain,
                        inner_radius=5.7,
                        outer_radius=6,
                        number_of_grains=50,
                        tool_pose=Pose.identity())

tool_creation_time1 = time.time()
print("tool_creation_time: ", tool_creation_time1-tool_creation_time0)


# plot the tool and grain
plot_grain(grain, GrainPlotConfig.default())
plot_tool(tool, ToolPlotConfig.default(tool))


# ------------------------- Workpiece creation --------------------------------
# Define workpiece type and dimensions
wp_creation_time0 = time.time()
box = Box.from_sizes(size_x=10, size_y=8,
                     size_z=10, pose=Pose.identity())
wp = Workpiece.from_box(box, spatial_resolution=2)

# Plot workpiece
plot_workpiece(wp, WorkpiecePlotConfig.default(wp))
wp_creation_time1 = time.time()
print("wp_creation_time: ", wp_creation_time1-wp_creation_time0)


# ------------------------ Kinematics creation ---------------------------------
#  Define kinematics
feed_rate = 10  # mm
start_point = Vector(8, 4, 16)
rotation_axis = Vector.e_x()
rotational_speed = 2*np.pi*(11500/60)
feed = Vector(0, 0, -feed_rate)  # mm/s
end_time = 1.3   # sec
time_step_size = 2E-4  # end_time/1000

kin_creation_time0 = time.time()
kin = Kinematics.from_feedrate_and_rotation_speed(start_point, rotation_axis,
                                                  rotational_speed, feed,
                                                  end_time, time_step_size)
kin_creation_time1 = time.time()
print("kin_creation_time: ", kin_creation_time1-kin_creation_time0)

# Plot kinematics
plot_kinematics(kin, KinematicsPlotConfig.default())


# ------------------------- Process Plot Options --------------------------------
# # visualize process
# process_creation_time0 = time.time()
# process = Process(tool, wp, kin)
# plot_process(process, ProcessPlotConfig.default(process))
# process_creation_time1 = time.time()
# print("process_creation_time: ", process_creation_time1-process_creation_time0)


# # visualize grain trajectory
# plot_grain_trajectory(process=process,
#                       grain_index=0,
#                       plot_config=ProcessPlotConfig.default(process))


# --------------------------- Simulation Setup ---------------------------------
# Setting up Simulation
mat_remover_config = MaterialRemovalConfig(
    False, False, True, CollisionDetectionConfig.for_partial_tool_wp_contact_process(tool, wp, kin))

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
if animate:
    workpieces: List[Workpiece] = []
    tools: List[Tool] = []
    kin_poses: List[Pose] = []

current_time = -time_step_size


# ---------------------------- Simulation Loop ----------------------------------
for idx, current_tool_pose in enumerate(kin.ToolTrajectory.poses):
    current_time += time_step_size
    print(
        f"Current Time: {current_time:.4f} :: End Time: {end_time:.4f} :: Percentage: {(100 * current_time / end_time):.2f} %")
    # remove material
    mat_remover_result = \
        matRemover.update(wp,
                          tool,
                          current_tool_pose,
                          mat_remover_config)

    # apply physical models
    grain_force_model_result = grain_force_model(mat_remover_result,
                                                 tool,
                                                 force_model_config)
    tool_force_model_result = tool_force_model(grain_force_model_result,
                                               tool)
    # apply wear
    wear_model_result = apply_attritious_wear_model(
        tool, wear_magnitude_as_penetration_depth_percentage, mat_remover_result)

    apply_attritious_wear(wear_model_result, tool,
                          mat_remover_result, False)

    # Collecting wp/tool instances for animation (every 50 steps)
    if animate:
        if idx % 50 == 1:
            workpieces.append(copy.deepcopy(wp))
            tools.append(copy.deepcopy(tool))
            kin_poses.append(copy.deepcopy(current_tool_pose))

    # capture results in a list
    tool_force_results.append(tool_force_model_result)
    grain_force_results.append(grain_force_model_result)
    wear_model_results.wears_vector_list.append(
        wear_model_result.wears_vector)
    wear_model_results.wears_magnitude_list.append(
        wear_model_result.wears_magnitude)


# --------------------------- Result Saving ------------------------------------
# convert to handier format
grain_forces_array = GrainForceModelResultListNumpyArrays.from_grain_force_model_result_list(
    grain_force_results)  # Forces in grain_frame
tool_forces_array = ToolForceModelResultListFloat.from_tool_force_model_result_list(
    tool_force_results)  # Forces in tool frame
global_forces_array = tool_forces_array.get_forces_in_global(
    kin.ToolTrajectory)  # Forces in global frame

# save to disk
path = r"./SimulationResults"
wp.save_to_disk(os.path.join(path, 'wp_after_grinding.pkl'))
tool.save_to_disk(os.path.join(path, 'tool_after_grinding.pkl'))
kin.save_to_disk(os.path.join(path, 'kin.pkl'))
grain_forces_array.save_to_disk(os.path.join(path, 'grain_forces.pkl'))
tool_forces_array.save_to_disk(os.path.join(path, 'tool_forces.pkl'))
global_forces_array.save_to_disk(os.path.join(path, 'global_forces.pkl'))
wear_model_results.save_to_disk(
    os.path.join(path, 'wear_model_results.pkl'))


# ---------------------------- Animation Option ----------------------------------
if animate:
    tool_forces = tool_forces_array.get_forces_in_global(kin.ToolTrajectory)
    tool_forces_abs = np.sqrt(np.square(tool_forces.tool_forces_x)
                              + np.square(tool_forces.tool_forces_y)
                              + np.square(tool_forces.tool_forces_z))

    anim_kin = Kinematics(PoseTrajectory(
        [time for time in kin.ToolTrajectory.times if time % 50 == 1], kin_poses))

    # Function for animation
    animation_path = r"./My_simulation.mp4"
    animate_process(kin, tool_forces_abs, tools, workpieces,
                    anim_kin, save_path=animation_path)


# ----------------------------- End of Simulation ---------------------------------
program_time_1 = time.time()
print("Program Time: ", program_time_1 - program_time_0)
