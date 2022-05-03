"""The files provides the tutorial script for constructing objects"""

from SimulationToolbox.PhysicalObjects.tool import Tool
from SimulationToolbox.PhysicalObjects.grain import *
from SimulationToolbox.Geometry.geometric_objects import *
from SimulationToolbox.Geometry.geometry import *
from SimulationToolbox.Visualization.visualization_objects import *
from SimulationToolbox.Visualization.visualization import *


def tutorial_run():
    # from factory method
    grain_1 = Grain.get_box_grain(
        size=0.2, pose=Pose.from_translation(Vector(2, 0, 0)))
    plot_grain(grain_1, GrainPlotConfig.default())

    # from stl file
    path = "./Tutorials/Resources/Grains/Cuboctahedron.stl"
    grain_2 = Grain.from_stl(path=path,
                             grain_pose=Pose.identity(),
                             mesh_pose=Pose.identity())
    plot_grain(grain_2, GrainPlotConfig.default())


# 1. Construct grain
    # manually define a grain
    # construct a mesh for the grain
    vertices = [Vector.origin(), Vector(1, 0, 0), Vector(0, 0, 1),
                Vector(1, 1, 0), Vector(0, 1, 0)]

    triangle_indices = [[0, 1, 2], [2, 1, 3], [
        2, 3, 4], [0, 2, 4], [0, 1, 3], [0, 3, 4]]

    mesh = Mesh(vertices, triangle_indices, Pose.identity())
    # construct a grain with a mesh
    grain_3 = Grain(Pose.identity(), mesh)
    plot_grain(grain_3, GrainPlotConfig.default())

# 2. Constructing tools
    # tool from factory method
    path = "./Tutorials/Resources/Grains/Cuboctahedron.stl"
    grain = Grain.from_stl(path=path,
                           grain_pose=Pose.identity(),
                           mesh_pose=Pose.identity())

    tool = Tool.from_radius(basic_grain=grain,
                            inner_radius=6,
                            outer_radius=9,
                            number_of_grains=50,
                            tool_pose=Pose.identity())

    plot_tool(tool, ToolPlotConfig.default(tool))

    # manually defina a tool
    path = "./Tutorials/Resources/Grains/Cuboctahedron.stl"
    grain = Grain.from_stl(path=path,
                           grain_pose=Pose.identity(),
                           mesh_pose=Pose.identity())
    grain_1 = grain.scale(0.5).move(Pose.from_translation(Vector(-1, -1, 1)))
    grain_2 = grain.scale(0.1).move(Pose.from_translation(Vector(1, 1, 1)))
    grains = [grain_1, grain_2]

    # create tool with list of grains
    tool_2 = Tool(grains=grains,
                  pose=Pose.identity(),
                  rotation_axis=Vector.e_z())

    # move the tool to a certain pose
    tool_2 = tool_2.move(Pose.from_rotation_axis_angle(Vector.e_x(), -np.pi/8))
    # Plotting Tool
    plot_tool(tool_2, ToolPlotConfig.default(tool_2))

    # 3. Construct workpiece
    # workpiece from box
    pose = Pose.from_rotation_axis_angle(Vector.e_z(), -np.pi/8)
    box = Box.from_sizes(size_x=5, size_y=5, size_z=5, pose=pose)
    workpiece = Workpiece.from_box(box, spatial_resolution=1)
    plot_workpiece(workpiece, WorkpiecePlotConfig.default(workpiece))
    # workpiece from cylinder
    cylinder = Cylinder(radius=5.0,
                        z_min=-5.0,
                        z_max=5.0,
                        pose=Pose.identity())

    workpiece = Workpiece.from_cylinder(cylinder,
                                        spatial_resolution=1)
    plot_workpiece(workpiece, WorkpiecePlotConfig.default(workpiece))

    position = Vector(0, 0, 0)
    pose = Pose.from_rotation_first_then_translation(
        Vector.e_z(), np.pi/4.0, position)
    z_points = 5*(-np.array([1, 1, 1.4, 1.5, 1.4, 1, 1, 2, 2])+2)
    x_points = 5*(np.array([1, 1.3, 1.35, 1.5, 1.65, 1.7, 2, 2, 1])-1)
    profile = FlatManifold.from_x_z_arrays(
        x_points, z_points, Pose.identity())
    extruded_volume = ExtrudedVolume(profile, extrusion_length=10)

    workpiece = Workpiece.from_extruded_volume(
        extruded_volume, spatial_resolution=0.5)
    plot_workpiece(workpiece, WorkpiecePlotConfig.default(workpiece))

    # manually define a workpiece
    list_manifold = []
    for i in range(10):
        position = Vector(0.5, 0.5 + i*0.1, 0)
        pose = Pose.from_rotation_first_then_translation(
            Vector.e_z(), i*np.pi/20.0, position)
        z_points = 5*(-np.array([1, 1, 1.4, 1.5, 1.4, 1, 1, 2, 2])+2)
        x_points = 5*(np.array([1, 1.3, 1.35, 1.5, 1.65, 1.7, 2, 2, 1])-1)
        list_manifold.append(
            FlatManifold.from_x_z_arrays(x_points, z_points, pose))

    workpiece = Workpiece(list_manifold, Pose.identity())
    plot_workpiece(workpiece, WorkpiecePlotConfig.default(workpiece))

    # 4. Construct Kinematic
    #  From feedrate and rotation speed
    kin_1 = Kinematics.\
        from_feedrate_and_rotation_speed(start_point=Vector(0, 1, 0),
                                         rotation_axis=Vector.e_z(),
                                         rotational_speed=np.pi,
                                         feed=Vector(1, 1, 0).
                                         normalize().scale(3),
                                         end_time=5,
                                         time_step_size=0.1)
    plot_kinematics(kin_1, KinematicsPlotConfig.default())

    # Construct a rollercoaster-like rigid Kinematics
    times = [1, 2, 3, 4, 5]
    poses = []
    times = np.linspace(0, 1.5*np.pi, 100)
    for t in times:
        position = Vector(np.cos(t), np.sin(t), 0.5*np.sin(4*t)).scale(10)
        poses.append(Pose.from_translation(position))
    trajectory = PoseTrajectory(times, poses)
    kin_2 = Kinematics(trajectory)
    plot_kinematics(kin_2, KinematicsPlotConfig.default(), pose_scale=1)


if __name__ == "__main__":
    configuration.do_plot = True
    tutorial_run()
