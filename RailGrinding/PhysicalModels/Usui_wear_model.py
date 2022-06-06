"""This module is devoted to the calculation of wear of grains ."""

from typing import *

from SimulationToolbox.Simulation.material_removal import *
from SimulationToolbox.Simulation.wear_models import *

# Example wear model for wear model core tests
from PhysicalModels.takazawa_temperature_model import GrainTemperatureModelResult


def wear_magnitude_as_penetration_depth_percentage(penetration_depth: float, percentage: float = 1) -> float:
    return penetration_depth * percentage / 100


def wear_magnitude_as_penetration_depth_percentage(grain_id,
                                                   temperature_model_result,
                                                   ambient_temperature,
                                                   arrhenius_constant,
                                                   specific_cutting_force,
                                                   mat_remover_result,
                                                   simulation_time_step,
                                                   wear_factor) -> float:
    grain_speed = 0.001 * mat_remover_result.grains_displacement[grain_id].norm() / simulation_time_step  # [m/s]
    cutting_force_magnitude = specific_cutting_force * mat_remover_result.projected_areas[grain_id]  # [N]
    grinding_temperature = ambient_temperature + temperature_model_result.grains_temperature_rise[grain_id]
    wear = wear_factor * cutting_force_magnitude * grain_speed * np.exp(
        -(arrhenius_constant / (grinding_temperature))) * simulation_time_step
    # print("percentage", wear)
    # print("grain_speed", grain_speed)
    # print("simulation_time_step", simulation_time_step)
    # print("cutting_force_magnitude", cutting_force_magnitude)
    # print("grinding_temperature", grinding_temperature)
    return wear


class SimulationWearModelResults:
    simulation_wear_model_results: List[AttritiousWearModelResult]

    def __init__(self, attritious_wear_result_list: List[AttritiousWearModelResult]):
        self.simulation_wear_model_results = attritious_wear_result_list

    def save_to_disk(self, file_name: str):
        with open(file_name, 'wb') as file:
            dill.dump(self, file)

    @staticmethod
    def load_from_disk(file_name):
        with open(file_name, 'rb') as file:
            object = dill.load(file)
        if isinstance(object, SimulationWearModelResults):
            return object
        else:
            Exception(
                "The loaded SimulationWearModelResults object is not from the current version of this class.")


def apply_attritious_wear_model(tool: Tool, wear_magnitude_model: Callable[[float], float],
                                mat_remover_result: MaterialRemovalResult,
                                temperature_model_result: GrainTemperatureModelResult,
                                ambient_temperature,
                                workpiece_material_propertise,
                                arrhenius_constant,
                                specific_cutting_force,
                                time_step_size,
                                wear_factor) -> AttritiousWearModelResult:
    wears_vector = []
    wears_magnitude = []
    for grain_idx, grain in enumerate(tool.grains):

        if mat_remover_result.penetration_depths[grain_idx] < default_precision:
            wear_magnitude = 0
            wear_vector = Vector.origin()
        else:
            wear_magnitude = wear_magnitude_model(grain_idx,
                                                  temperature_model_result,
                                                  ambient_temperature,
                                                  arrhenius_constant,  # arrhenius_constant
                                                  specific_cutting_force,  # [N/mm^2]
                                                  mat_remover_result,
                                                  time_step_size,  # [s]
                                                  wear_factor)  # unit-less
            # for version 1.2.0
            # wear_vector = mat_remover_result.surface_normals[grain_idx].normalize().scale(wear_magnitude)

            wear_vector = mat_remover_result.wear_directions[grain_idx].normalize().scale(wear_magnitude)
        wears_magnitude.append(wear_magnitude)
        wears_vector.append(wear_vector)

    return AttritiousWearModelResult(wears_vector, wears_magnitude)
