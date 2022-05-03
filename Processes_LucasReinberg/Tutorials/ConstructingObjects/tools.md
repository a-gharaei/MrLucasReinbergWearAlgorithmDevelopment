# How to construct a tool

A tool consists is realized by grains in a list, a Pose , a rotation axis and vector in normal direction to cutting surface.:

```python
class Tool:
    pose: Pose
    grains: List[Grain]
    rotation_axis: Vector
    cutting_surface_normal: Vector
```

The code of this tutorial can be found here: [construction_objects.py](./../../Tutorials/ConstructingObjects/construction_objects.py)


- [How to construct a tool](#how-to-construct-a-tool)
  - [Tool from factory method](#tool-from-factory-method)
  - [Manually construct a tool](#manually-construct-a-tool)



## Tool from factory method

Factory methods can be used to get specific tools out of the box, without the need to manually define and place grains.  Using the factory method tool construction can be expanded to various forms and shapes depending on various use-case scenario:

```python
# tool from factory method
# create a grain
path = "./SimulationToolbox/PhysicalObjects/stl_file/Cuboctahedron.stl"
grain = Grain.from_stl(path=path,
                        grain_pose=Pose.identity(),
                        mesh_pose=Pose.identity())

# create tool with the grain
tool = Tool.from_diameter(basic_grain=grain,
                          inner_diameter=6,
                          outer_diameter=9,
                          number_of_grains=50,
                          tool_pose=Pose.identity())
# plot result
plot_tool(tool, ToolPlotConfig.default(tool))
```

![tool_factory_method](./../../Resources/Images/Tutorials/ConstructingObjects/tool_from_factory_method.PNG)

> **Note:** The tool Pose is defined in the global Frame. The rotation axis is defined in tool Frame, which is by default set to the Z-axis (blue) The cutting surface normal is set to the - Z direction as default, representing the rail grinding case. For other types onf processes the noral direction need to be specified.

## Manually construct a tool

Here a tool is constructed out of two grains of with different sizes at locations within the tool frame:

```python
# manually define a tool
path = "./SimulationToolbox/PhysicalObjects/stl_file/Cuboctahedron.stl"
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
tool_2 = tool_2.move(Pose.from_rotation_axis_angle(Vector.e_x(), np.pi/8))

# Plotting Tool
plot_tool(tool_2, ToolPlotConfig.default(tool_2))
```

![tool_manual](./../../Resources/Images/Tutorials/ConstructingObjects/manually_defined_tool.PNG)
