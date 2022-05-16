
from SimulationToolbox.Simulation.physical_models import *
from SimulationToolbox.Simulation.material_removal import MaterialRemovalResult
import dill


class MaterialRemovalResultsList:
    material_removal_result_list: List[MaterialRemovalResult]

    def __init__(self, material_removal_result: List[MaterialRemovalResult]):
        self.material_removal_result_list = material_removal_result

    def save_to_disk(self, file_name: str):
        with open(file_name, 'wb') as file:
            dill.dump(self, file)

    @staticmethod
    def load_from_disk(file_name):
        with open(file_name, 'rb') as file:
            object = dill.load(file)
        if isinstance(object, MaterialRemovalResultsList):
            return object
        else:
            Exception(
                "The loaded MaterialRemovalResultList object is not from the current version of this class.")


class GrainForceModelConfiguration:
    specific_cutting_force: float
    grinding_force_ratio: float

    def __init__(self, specific_cutting_force: float, grinding_force_ratio: float) -> None:
        self.specific_cutting_force = specific_cutting_force
        self.grinding_force_ratio = grinding_force_ratio


def grain_force_model(mat_remover_result: MaterialRemovalResult,
                      tool: Tool, config: GrainForceModelConfiguration) \
        -> GrainForceModelResult:
    # calculate_velocity_vector of grains in tool-frame

    result = GrainForceModelResult.from_all_none(tool)

    active_grain_index = [index for index, volume in enumerate(
        mat_remover_result.removed_volumes) if volume - default_precision > 0]

    active_grain_displacement_vectors = np.asarray(mat_remover_result.grains_displacement)[
        active_grain_index].tolist()
    active_grain_displacement_in_tool_frame: List[Vector] = Vector.change_frame_vector_list(
        active_grain_displacement_vectors, [], [tool.pose])

    for index, active_grain_idx in enumerate(active_grain_index):

        # calculate forces
        cutting_force_magnitude = config.specific_cutting_force * mat_remover_result.projected_areas[active_grain_idx]
        normal_force_magnitude = cutting_force_magnitude / config.grinding_force_ratio

        if active_grain_displacement_in_tool_frame[index].norm() == 0:
            cutting_force = active_grain_displacement_in_tool_frame[index].scale(-1)
        else:
            cutting_force = active_grain_displacement_in_tool_frame[index]. \
                normalize().scale(-cutting_force_magnitude)

        normalized_rotation_axis = tool.rotation_axis

        # If normal force is along rotation axis:
        normal_force_direction = normalized_rotation_axis.change_frame_vector(
            [], [tool.grains[active_grain_idx].pose])  # force represented in grain frame
        # TODO: check how a tilt or approach could potentially change the calculation of the normal force
        # If normal force is towards rotation axis (from grain rotation center):
        # rotation_center_of_grain = normalized_rotation_axis.scale(
        #     normalized_rotation_axis.dot(tool.grains[grain_id].get_position()))
        # radius_vector = tool.grains[grain_id].get_position().subtract(
        #     rotation_center_of_grain)
        # normal_force_direction = radius_vector.scale(
        #     -1).change_frame_vector([], [tool.grains[grain_id].pose])

        normal_force = normal_force_direction.scale(normal_force_magnitude)

        total_grain_force = cutting_force.add(normal_force)
        # for release 1.2.0
        # result.grain_forces[active_grain_idx] = GrainForce(total_grain_force)

        result.grain_forces[active_grain_idx] = GrainForce(total_grain_force, cutting_force, normal_force)

    return result
