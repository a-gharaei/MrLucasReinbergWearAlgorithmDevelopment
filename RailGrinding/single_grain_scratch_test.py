"""This scripts serves as a tutorial for users to get familiar with iBrus.
It will show case how to define various objects needed and how to define
and run a grinding process."""
import os
import sys

import matplotlib.animation as animation
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402
from numpy import True_, mat
from SimulationToolbox.PhysicalObjects import grain  # noqa: E402
from SimulationToolbox.PhysicalObjects.grain import *  # noqa: E402
from SimulationToolbox.PhysicalObjects.tool import *  # noqa: E402
from SimulationToolbox.Simulation.kinematics import Kinematics  # noqa: E402
from SimulationToolbox.Simulation.material_removal import *  # noqa: E402
from SimulationToolbox.Simulation.process import *  # noqa: E402
from SimulationToolbox.Utilities.SimulationToolboxConfiguration import (  # noqa: E402
    configuration,
)
from SimulationToolbox.Visualization.animation import animate_process  # noqa: E402
from SimulationToolbox.Visualization.visualization import *  # noqa: E402
from SimulationToolbox.Visualization.visualization_objects import (  # noqa: E402
    GrainPlotConfig,
    ToolPlotConfig,
)

from PhysicalModels import *  # noqa: E402
from PhysicalModels.kienzle_force_model import *
from wear_models import *  # noqa: E402


def animation_data(composition: VisualComposition):

    vertices_for_scale = []
    data = []
    for polygon in composition.polygons:
        current_polygon_vertices = []
        for vertex in polygon.vectors:
            current_polygon_vertices.append(vertex.value[:3])
        vertices_for_scale.extend(current_polygon_vertices)
        config = polyplotconfig_to_matplotlib(polygon.plot_config)
        current_polygon = mplot3d.art3d.Poly3DCollection(
            [current_polygon_vertices],
            facecolors=config["face_color"],
            edgecolors=config["edge_color"],
            linewidths=config["linewidth"],
            linestyles=config["linestyle"],
            alpha=config["alpha"],
        )

        data.append(current_polygon)

    for line in composition.lines:
        current_line_vertices = []
        for vertex in line.vectors:
            current_line_vertices.append(vertex.value[:3])
        vertices_for_scale.extend(current_line_vertices)
        config = lineplotconfig_to_matplotlib(line.plot_config)
        current_line = mplot3d.art3d.Line3DCollection(
            [current_line_vertices],
            colors=config["color"],
            linewidths=config["linewidth"],
            linestyles=config["linestyle"],
            alpha=config["alpha"],
        )
        data.append(current_line)

    return [data, vertices_for_scale]


fracture_wear_results = [[], [], []]
fractured_grains = []
fracture_data = []
workpiece_states = []
fracture_wear = True
# Single grain scratch test
use_reduced_grain = False
animate = True
path = r"Resources/Grains/Cuboctahedron.stl"
grain_1 = Grain.from_stl(
    path,
    Pose.from_translation_first_then_rotation(
        Vector(10, 0, 0), Vector.e_z(), -np.pi / 2
    ),
    Pose.identity(),
).scale(1.5)

tool = Tool([grain_1], Pose.identity())

grain_plot_config = GrainPlotConfig.default()
tool_plot_config = ToolPlotConfig.default(tool)
plot_grain(grain_1, grain_plot_config)
plot_tool(tool, tool_plot_config)
plt.show()


box = Box.from_sizes(
    size_x=7.5,
    size_y=25,
    size_z=5,
    pose=Pose.from_rotation_axis_angle(Vector.e_z(), -np.pi),
)

wp = Workpiece.from_box(box, spatial_resolution=0.5)
plot_workpiece(wp, WorkpiecePlotConfig.default(wp))


feed_rate = 0
start_point = Vector(-2.5, -12, 14)
rotation_axis = Vector.e_x()
rotational_speed = np.pi / 180
feed = Vector(feed_rate, 0, 0)
end_time = 150
time_step_size = 0.5

kin = Kinematics.from_feedrate_and_rotation_speed(
    start_point, rotation_axis, rotational_speed, feed, end_time, time_step_size
)

collider_wp_proximity = kin.get_max_travelled_distance_of_grains(
    tool, kin.ToolTrajectory
)
max_grain_bounding_sphere_radius = tool.get_max_bounding_sphere_radius()
start_tool_pose = kin.ToolTrajectory.poses[0]

collider_config = CollisionDetectionConfig.for_partial_tool_wp_contact_process(
    tool, wp, kin
)

mat_removal_config = MaterialRemovalConfig(
    False,
    True,
    False,
    collider_config,
)
matRemover = MaterialRemover(wp, tool, start_tool_pose, kin, mat_removal_config)


force_model_config = GrainForceModelConfiguration(1, 0.3)
grain_force_results = []
tool_force_results = []

workpieces: List[Workpiece] = []
tools: List[Tool] = []
grains: List[Grain] = []

old_grains = []
for grain in tool.grains:
    old_grains.append(copy.deepcopy(grain))

le_index_list = []

for current_tool_pose in kin.ToolTrajectory.poses:

    mat_remover_result = matRemover.update(
        wp, tool, current_tool_pose, mat_removal_config
    )

    wear_model_result = apply_attritious_wear_model(
        tool, wear_magnitude_as_penetration_depth_percentage, mat_remover_result
    )
    le_index_list.append(mat_remover_result.leading_vertex_indices)

    workpieces.append(copy.deepcopy(wp))
    tools.append(copy.deepcopy(tool))
    grains.append(copy.deepcopy(grain_1).move(Pose.from_translation(Vector(0, 0, 0))))

    grain_force_model_result = grain_force_model(
        mat_remover_result, tool, force_model_config
    )
    tool_force_model_result = tool_force_model(grain_force_model_result, tool)

    tool_force_results.append(tool_force_model_result)
    grain_force_results.append(grain_force_model_result)
    if fracture_wear:
        (
            total_removed_volume,
            rankine_stresses,
            penetration_depths,
            fractured_grains_per_timestep,
            fracture_informations,
        ) = apply_fracture_wear_model(
            tool, mat_remover_result, grain_force_model_result
        )
        if rankine_stresses:
            rankine_stress_mean = mean(rankine_stresses)
            fracture_wear_results[1].append(rankine_stress_mean)
        if penetration_depths:
            penetration_depths_mean = mean(penetration_depths)
            fracture_wear_results[2].append(penetration_depths_mean)
        fracture_wear_results[0].append(total_removed_volume)
        fractured_grains.extend(fractured_grains_per_timestep)
        fracture_data.extend(fracture_informations)


grain_forces_array = (
    GrainForceModelResultListNumpyArrays.from_grain_force_model_result_list(
        grain_force_results
    )
)  # Forces in grain_frame
tool_forces_array = ToolForceModelResultListFloat.from_tool_force_model_result_list(
    tool_force_results
)  # Forces in tool frame
global_forces_array = tool_forces_array.get_forces_in_global(
    kin.ToolTrajectory
)  # Forces in global frame


path = r"single_grain_scratch_test/Results"
wp.save_to_disk(os.path.join(path, "wp_after_grinding.pkl"))
tool.save_to_disk(os.path.join(path, "tool_after_grinding.pkl"))
kin.save_to_disk(os.path.join(path, "kin.pkl"))
grain_forces_array.save_to_disk(os.path.join(path, "grain_forces.pkl"))
tool_forces_array.save_to_disk(os.path.join(path, "tool_forces.pkl"))
global_forces_array.save_to_disk(os.path.join(path, "global_forces.pkl"))

df = pd.DataFrame(le_index_list)
df.to_csv(os.path.join(path, "le_indices_reduced.csv"))

plt.close("all")
old_grains.extend(tool.grains)
plot_sample_tool = Tool(old_grains, Pose.from_translation(Vector(0, 10, 0)))
blue_config = [
    GrainPlotConfig.from_color(Color.white()) for i in range(len(tool.grains))
]
red_config = [
    GrainPlotConfig.from_color(Color.black()) for i in range(len(tool.grains))
]
blue_config.extend(red_config)
plot_tool(
    plot_sample_tool, ToolPlotConfig(blue_config), library=PlotLibrary.matplot_lib
)
plt.show()
plot_workpiece(wp, WorkpiecePlotConfig.default(wp))
plt.show()


# Animation

if animate:
    title = "Process Animation"
    axis_labels = ["x axis", "y axis", "z axis"]
    length_unit = "(m)"
    save_path = r"animations/Single_grain_scratch_simulation.mp4"

    fig = plt.figure(figsize=[18, 12])
    widths = [1.5, 1.5]
    heights = [1]
    spec = fig.add_gridspec(
        ncols=2, nrows=1, width_ratios=widths, height_ratios=heights
    )
    ax_1 = fig.add_subplot(spec[0, 0], projection="3d")

    state_of_sim = animation_data(
        visualize_state_of_simulation(
            tools[0].move(kin.ToolTrajectory.poses[0]),
            workpieces[0],
            StatePlotConfig.default(
                tools[0].move(kin.ToolTrajectory.poses[0]), workpieces[0]
            ),
        )
    )
    faces = state_of_sim[0]
    for face in faces:
        ax_1.add_collection3d(face)
    scale = np.array(state_of_sim[1]).flatten("F")
    ax_1.auto_scale_xyz(scale, scale, scale)

    ax_1.set_title(title, fontsize=18)
    ax_1.set_xlabel(axis_labels[0] + " " + length_unit, fontsize=18)
    ax_1.set_ylabel(axis_labels[1] + " " + length_unit, fontsize=18)
    ax_1.set_zlabel(axis_labels[2] + " " + length_unit, fontsize=18)
    ax_1.set_xlim([-15, 5])
    ax_1.set_ylim([-15, 5])
    ax_1.set_zlim([-15, 5])

    ax_2 = fig.add_subplot(spec[0, 1], projection="3d")
    ax_2.view_init(azim=110)
    state_of_sim = animation_data(visualize_grain(grains[0], GrainPlotConfig.default()))

    faces = state_of_sim[0]
    for face in faces:
        ax_2.add_collection3d(face)
    scale = np.array(state_of_sim[1]).flatten("F")
    ax_2.auto_scale_xyz(scale, scale, scale)

    ax_2.set_title("Grain wear progression", fontsize=18)
    ax_2.set_xlabel(axis_labels[0] + " " + length_unit, fontsize=18)
    ax_2.set_ylabel(axis_labels[1] + " " + length_unit, fontsize=18)
    ax_2.set_zlabel(axis_labels[2] + " " + length_unit, fontsize=18)

    fig.tight_layout()

    def update_figure(figure_no: int):
        state_of_sim_1 = animation_data(
            visualize_state_of_simulation(
                tools[figure_no].move(kin.ToolTrajectory.poses[figure_no]),
                workpieces[figure_no],
                StatePlotConfig.default(
                    tools[figure_no].move(kin.ToolTrajectory.poses[figure_no]),
                    workpieces[figure_no],
                ),
            )
        )
        faces_1 = state_of_sim_1[0]
        ax_1.collections = []
        for face in faces_1:
            ax_1.add_collection3d(face)

        state_of_sim_2 = animation_data(
            visualize_grain(grains[figure_no], GrainPlotConfig.default())
        )
        faces_2 = state_of_sim_2[0]
        ax_2.collections = []
        for face in faces_2:
            ax_2.add_collection3d(face)

    process_animation = animation.FuncAnimation(
        fig,
        update_figure,
        frames=range(0, len(kin.ToolTrajectory.poses)),
        interval=200,
        blit=False,
        repeat=False,
    )

    # plt.show()

    writer = animation.FFMpegWriter(fps=7, bitrate=7000)
    process_animation.save(save_path, writer=writer)
