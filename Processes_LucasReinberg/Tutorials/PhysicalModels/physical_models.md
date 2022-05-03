# Physical Models

## 1\. Introduction

Physical models in iBrus refers to the process specific equations and behavior that researchers need to implement to create physical foundation of their simulations. Some examples of these are force, wear and temperature models. These models are impmeneted into iBrus code in two parts: core models and process models. 

Core models are the functions and algorithm that are the base for all process specific models. They include only the physics that are expected to be the common points of all possible process models and act as an interface between process simulation and process specific models.

Process models refer to the physical phenomena that are speficic to a certain process. These are the equations and algorithms that need to be defined by the users. These models utilize the methods and attributes of core models to access and work with process simulation.


## 2\. Force Model

This is the model to calculate forces acting on the grains and tool during a process. Grain forces are process dependent, the magnitude and direction of the forcess need to be defined/calculated with the methods created by the users. An example model for rail grinding process is defined, which uses projected cutting area from simulation and specific cutting force/grindsing force ratio from user defined model configuration as inputs for force calculation:

```python
def grain_force_model(mat_remover_result: MaterialRemovalResult,
                      tool: Tool, config: GrainForceModelConfiguration)\
        -> GrainForceModelResult:
    '''The force result of this function ideally would be represented in grain frame. For optimization purposes, it's reprsented in tool frame'''

    # calculate_displacement_vector of grains in grain_frames
    result = GrainForceModelResult.from_all_none(tool)

    active_grain_index = [index for index, volume in enumerate(
        mat_remover_result.removed_volumes) if volume - default_precision > 0]

    active_grain_displacement_vectors = np.asarray(mat_remover_result.grains_displacement)[
        active_grain_index].tolist()
    active_grain_displacement_in_tool_frame: List[Vector] = Vector.change_frame_vector_list(
        active_grain_displacement_vectors, [], [tool.pose])

    for index, active_grain_idx in enumerate(active_grain_index):

        # calculate forces
        cutting_force_magnitude = config.specific_cutting_force * \
            mat_remover_result.projected_areas[active_grain_idx]
        normal_force_magnitude = cutting_force_magnitude / \
            config.grinding_force_ratio

        if active_grain_displacement_in_tool_frame[index].norm() == 0:
            cutting_force = active_grain_displacement_in_tool_frame[index].scale(
                -1)
        else:
            cutting_force = active_grain_displacement_in_tool_frame[index].\
                normalize().scale(-cutting_force_magnitude)

        # If normal force is along rotation axis:
        # force represented in tool frame
        normal_force_direction = tool.cutting_surface_normal.scale(-1)

        # # If normal force is towards rotation axis (from grain rotation center):
        # rotation_center_of_grain = tool.cutting_surface_normal.scale(-1).scale(
        #     tool.cutting_surface_normal.scale(-1).dot(tool.grains[grain_id].get_position()))
        # radius_vector = tool.grains[grain_id].get_position().subtract(
        #     rotation_center_of_grain)
        # normal_force_direction = radius_vector.scale(
        #     -1)

        normal_force = normal_force_direction.scale(normal_force_magnitude)

        total_grain_force = cutting_force.add(normal_force)
        result.grain_forces[active_grain_idx] = GrainForce(
            total_grain_force, cutting_force, normal_force)

    return result
```

The grain forces calculated by the user defined methods are used in calculation of tool forces. This method is a part of the core model, which simply adds up all grain forces by taking their position on tool into account to give the forces acting along each axis of tool frame, as well as the torque on the tool.


```python
def tool_force_model(grain_force_model_result: GrainForceModelResult,
                     tool: Tool) -> ToolForceModelResult:
    'The force results of this function are represented in tool frame'

    tool_force = Vector.origin()
    torque_around_rotation_axis = 0

    active_grain_index = [index for index,
                          grain_force in enumerate(grain_force_model_result.grain_forces) if grain_force is not None]

    # aggregate Forces & torques
    for grain_id in active_grain_index:
        # calculate forces represented in tool frame
        grain_force_in_tool_frame = grain_force_model_result.grain_forces[grain_id].\
            total_force

        normalized_rotation_axis = tool.rotation_axis
        rotation_center_of_grain = normalized_rotation_axis.scale(
            normalized_rotation_axis.dot(tool.grains[grain_id].get_position()))
        radius_vector = tool.grains[grain_id].get_position().subtract(
            rotation_center_of_grain)

        tool_force = tool_force.add(grain_force_in_tool_frame)

        torque_vector = radius_vector.cross(grain_force_in_tool_frame)
        torque_around_rotation_axis += -(torque_vector.dot(
            tool.rotation_axis))

    return ToolForceModelResult(torque_around_rotation_axis, tool_force)
```

The grain and tool force models can be integrated into a process simulation loop, after the material removal calculation as follows:

```python
# remove material
mat_remover_result = \
    matRemover.update(wp,
                        tool,
                        current_tool_pose,
                        maximum_distance_travelled_by_grain)
# apply physical models
grain_force_model_result = grain_force_model(mat_remover_result,
                                            tool,
                                            force_model_config)
tool_force_model_result = tool_force_model(grain_force_model_result,
                                            tool)
```

The complete process simulation example and methods to save force model results can be found here: [process_simulation_walkthrough.py](./../../Tutorials/ProcessSimualtionWalkthrough/process_simulation_walkthrough.py)


## 2\. Wear Model

This is the model to calculate and appy the wear on grains during a grinding process.

Currently, only the attritious wear and bonding wear/pullout are implemented as methods. In case of attritious wear, the core part of the wear model includes the algorithm to calculate the leading vertex among all grain mesh vertices, then finding a wear vector to move this vertex along the direction normal to the workpiece surface. The process part for this model takes a method to calculate magintude of wear as user input and scales the wear vector from the core part by this magnitude. The parameters required for model calculations, such as projected area and penetration depth can be obtained from the result of material removal.

```python
class MaterialRemovalResult:
    removed_volumes: List[float]  # indices matches indices of grains in tool
    projected_areas: List[float]  # indices matches indices of grains in tool
    penetration_depths: List[float]
    leading_vertex_indices: List[int]
    wear_directions: List[Vector]   
    grains_displacement: List[Vector] # represented in base_frame of tool and wp
```

An example for the process part of wear model is given for rail grinding:

```python
def wear_magnitude_as_penetration_depth_percentage(penetration_depth: float, percentage: float = 1) -> float:
    return penetration_depth*percentage/100


def apply_attritious_wear_model(tool: Tool, wear_magnitude_model: Callable[[float], float], mat_remover_result: MaterialRemovalResult) -> AttritiousWearModelResult:

    wears_vector = []
    wears_magnitude = []
    for grain_idx, grain in enumerate(tool.grains):

        if mat_remover_result.penetration_depths[grain_idx] == 0:
            wears_vector.append(Vector.origin())
            wears_magnitude.append(0)
            continue

        wear_magnitude = wear_magnitude_model(
            mat_remover_result.penetration_depths[grain_idx])
        wears_magnitude.append(wear_magnitude)
        wears_vector.append(
            mat_remover_result.wear_directions[grain_idx].normalize().scale(wear_magnitude))

    return AttritiousWearModelResult(wears_vector, wears_magnitude)
```

Here, the function magnitude calculation is defined as a callable, which gives 1 % of penetration depth as wear magnitude. This callable is used as an input for the wear model to obtain the wear vector. The users can also integrate their wear magnitude functions into wear model function with required parameters as inputs for the function. The result of this function is then used as an input for the core method `apply_attritious_wear()`:

```python
def apply_attritious_wear(wear_model_result: AttritiousWearModelResult, tool: Tool, mat_remover_result: 'MaterialRemovalResult', use_reduced_grain):

    if len(tool.grains) != len(wear_model_result.wears_vector):
        raise Exception('Wear model results does not match number of grains!')

    for grain_idx, grain in enumerate(tool.grains):
        if mat_remover_result.leading_vertex_indices[grain_idx] != -1:
            grain.get_mesh().vertices[mat_remover_result.leading_vertex_indices[grain_idx]] = \
                grain.get_mesh().vertices[mat_remover_result.leading_vertex_indices[grain_idx]].add(
                wear_model_result.wears_vector[grain_idx])
            if (grain.get_mesh().vertices[mat_remover_result.leading_vertex_indices[grain_idx]].norm() - grain.bounding_sphere_radius) > default_precision:
                grain.get_mesh().vertices[mat_remover_result.leading_vertex_indices[grain_idx]] = \
                    grain.get_mesh().vertices[mat_remover_result.leading_vertex_indices[grain_idx]].normalize(
                ).scale(grain.bounding_sphere_radius)

    if use_reduced_grain:
        modify_reduced_grains(tool, mat_remover_result)
```
After a vertex is moved, the method also checks if it exceeds grain bounding shpere radius to detect invalid geometries. If this is the case, the wear magnitude is normalized to keep vertices on the bounding sphere.

The grains are assumed to be floating in bonding material and the change of bonding around grain is controlled by center protrusion of grain, increasing with bond wear. This is equivalent to the assumption of tool surface height changing with bond wear. The magnitude of bond wear is calculated in process part with selected method, for example the Archard wear equation:

```python
def apply_archard_bonding_wear_model(tool: Tool, wear_coefficient: float, hardness: float, grain_force_model_result: GrainForceModelResult) ->  	          BondingWearModelResult:

    wears_magnitude = []
    wears_vector = []
    for grain_idx, grain in enumerate(tool.grains):

        if grain_force_model_result.grain_forces[grain_idx] is None or grain_force_model_result.grain_forces[grain_idx].normal_force.norm() == 0:
            wears_magnitude.append(0)
            wears_vector.append(Vector.origin())
            continue

        # Sliding wear equation from Archard
        bonding_wear = (wear_coefficient * grain_force_model_result.grain_forces[grain_idx].normal_force.norm())\
            / (hardness * 2*(grain.bounding_sphere_radius - grain.center_protrusion.norm()))
        wears_magnitude.append(bonding_wear)
        wears_vector.append(tool.cutting_surface_normal.scale(bonding_wear))

    return BondingWearModelResult(wears_vector, wears_magnitude)
```
The result of this model is used to change center protrusion of grain. Then grains are processees by a method to check if pullout condition is satisfied. A sample pullout condition implemented is the moment balance of grains. It checks the moments of cutting force and retention force. The retention force on a grain can be determined from bonding material strength or manually set as input.

```python
def apply_pullout_condition(tool: Tool, grain_force_model_result: 'GrainForceModelResult', mat_remover_result,
                            bonding_strength: float = 0, retention_force: float = 0):
    "For the pullout condition, set only one of following: Bonding strength or retention force."
    # TODO: Capability to take a set of retention force values and use them accordingly for changing bonding height

    for grain_idx, grain in enumerate(tool.grains):
        if mat_remover_result.removed_volumes[grain_idx] > default_precision:
            cutting_force = grain_force_model_result.grain_forces[grain_idx].cutting_force

            leading_edge_vertex = grain.get_mesh(
            ).vertices[mat_remover_result.leading_vertex_indices[grain_idx]]
            surface_normal_in_grain_frame = tool.cutting_surface_normal.change_frame_vector([
            ], [grain.pose])
            edge_distance_to_center = abs(leading_edge_vertex.dot(
                surface_normal_in_grain_frame.normalize())/surface_normal_in_grain_frame.norm())
            center_protrusion_magnitude = grain.center_protrusion.dot(
                tool.cutting_surface_normal)
            bonding_height = grain.bounding_sphere_radius - center_protrusion_magnitude

            # Limit force/strength to resist pullout
            # Equation below is taken from this dissertation: https://doi.org/10.3929/ethz-b-000426867
            # Page 130
            retention_force_required = cutting_force.norm() * \
                (6*(edge_distance_to_center + center_protrusion_magnitude) +
                 3*bonding_height)/(4*bonding_height)

            if bonding_strength > 0 and retention_force == 0:
                grain_bonding_area = 2*np.pi*grain.bounding_sphere_radius*bonding_height
                bonding_strength_required = retention_force_required/grain_bonding_area
                if bonding_strength < bonding_strength_required:
                    grain.pullout = True
            elif bonding_strength == 0 and retention_force > 0:
                if retention_force < retention_force_required:
                    grain.pullout = True
            else:
                raise Exception("None or both bonding_strength and retention_force are set above zero.  \
                    Please only use either bonding_strength or retention_force. ")
```

The wear models can be integrated into a process simulation loop, after the material removal calculation as follows:

```python
# remove material
mat_remover_result = \
    matRemover.update(wp,
                        tool,
                        current_tool_pose,
                        maximum_distance_travelled_by_grain)
# apply attritious wear
wear_model_result = apply_attritious_wear_model(
    tool, wear_magnitude_as_penetration_depth_percentage, mat_remover_result)
apply_attritious_wear(wear_model_result, tool, mat_remover_result)

# apply bond wear and pullout
bond_wear_model_result = apply_archard_bonding_wear_model(tool,
                                                            1, 1, grain_force_model_result)
apply_bonding_wear(bond_wear_model_result, tool)
apply_pullout_condition(
    tool, grain_force_model_result, mat_remover_result, bonding_strength = 0.05)
```

The complete process simulation example and methods to save wear model results can be found here: [process_simulation_walkthrough.py](./../../Tutorials/ProcessSimualtionWalkthrough/process_simulation_walkthrough.py)
