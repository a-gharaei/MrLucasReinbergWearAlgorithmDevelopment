from SimulationToolbox.PhysicalObjects.tool import *
from SimulationToolbox.Geometry.geometry import *
from SimulationToolbox.PhysicalObjects.grain import *
from SimulationToolbox.Simulation.material_removal import *
from SimulationToolbox.Simulation.physical_models import *


class GrainForceModelConfiguration:
    specific_cutting_force: float
    grinding_force_ratio: float

    def __init__(self, specific_cutting_force: float, grinding_force_ratio: float) -> None:
        self.specific_cutting_force = specific_cutting_force
        self.grinding_force_ratio = grinding_force_ratio


def grain_force_model(mat_remover_result: MaterialRemovalResult, tool: Tool, config: GrainForceModelConfiguration)\
        -> GrainForceModelResult:

    # calculate_velocity_vector of grains in grain_frames
    result = GrainForceModelResult([])
    for grain_id, velocity in enumerate(mat_remover_result.grains_displacement):
        if mat_remover_result.penetration_depths[grain_id] > 0:
            grain_displacements_in_grain_frame = velocity.change_frame_vector(
                [], [tool.pose, tool.grains[grain_id].pose])
            # calculate forces
            cutting_force_magnitude = config.specific_cutting_force * \
                mat_remover_result.projected_areas[grain_id]
            normal_force_magnitude = cutting_force_magnitude / \
                config.grinding_force_ratio

            if grain_displacements_in_grain_frame.norm() == 0:
                cutting_force = grain_displacements_in_grain_frame.scale(-1)
            else:
                cutting_force = grain_displacements_in_grain_frame.\
                    normalize().scale(-cutting_force_magnitude)

            normalized_rotation_axis = tool.rotation_axis.normalize()

            # If normal force is along rotation axis:
            normal_force_direction = normalized_rotation_axis.change_frame_vector(
                [], [tool.grains[grain_id].pose])

            # If normal force is towards rotation axis (from grain rotation center):
            # rotation_center_of_grain = normalized_rotation_axis.scale(
            #     normalized_rotation_axis.dot(tool.grains[grain_id].get_position()))
            # radius_vector = tool.grains[grain_id].get_position().subtract(
            #     rotation_center_of_grain)
            # normal_force_direction = radius_vector.scale(
            #     -1).change_frame_vector([], [tool.grains[grain_id].pose])

            normal_force = normal_force_direction.scale(normal_force_magnitude)

            total_grain_force = cutting_force.add(normal_force)
            result.grain_forces.append(GrainForce(
                total_grain_force, cutting_force, normal_force))
        else:
            result.grain_forces.append(GrainForce(
                Vector.origin(), Vector.origin(), Vector.origin()))
    return result
