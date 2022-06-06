"""This module is devoted to the calculation of grinding temperatures of grains ."""
from typing import List

import dill
import numpy as np
from Resources.Workpieces.workpiece_factory import WorkpieceMaterialProperties
from SimulationToolbox.Simulation.material_removal import MaterialRemovalResult
from SimulationToolbox.Utilities.helpers import (
    InputAreNotTheSameDataType,
    default_precision,
)


class GrainTemperatureModelResult:
    grains_temperature_rise: List[float]

    def __init__(self, grains_temperature_rise: List[float]):
        self.grains_temperature_rise = grains_temperature_rise

    def compare(self, other, precision: float):
        if isinstance(other, GrainTemperatureModelResult):
            for grain_id, grain_temperature in enumerate(self.grains_temperature_rise):
                grain_temperature.compare(other.grain_forces[grain_id], precision)
        else:
            raise InputAreNotTheSameDataType


class TemperatureResultList:
    temperature_results_list: List[GrainTemperatureModelResult]

    def __init__(self, temperature_results_list: List[GrainTemperatureModelResult]):
        self.temperature_results_list = temperature_results_list

    def save_to_disk(self, file_name: str):
        with open(file_name, 'wb') as file:
            dill.dump(self, file)

    @staticmethod
    def load_from_disk(file_name):
        with open(file_name, 'rb') as file:
            object = dill.load(file)
        if isinstance(object, TemperatureResultList):
            return object
        else:
            Exception(
                "The loaded TemperatureResultList object is not from the current version of this class.")


def calculate_grain_heat_flux(grain_id,
                              mat_remover_result,
                              specific_cutting_force,
                              contact_length, grain_speed):
    # calculate cutting_force_magnitude [N]
    cutting_force_magnitude = specific_cutting_force * mat_remover_result.projected_areas[grain_id]

    if mat_remover_result.penetration_depths[grain_id] > default_precision:
        # calculate grain_cutting_width or bw [m] based on the assumption that the penetrated shape is a triangle
        grain_cutting_width = 2 * (mat_remover_result.projected_areas[grain_id] * 0.001 * 0.001) / \
                              (mat_remover_result.penetration_depths[grain_id] * 0.001)
        heat_flux = (cutting_force_magnitude * grain_speed) / (contact_length * grain_cutting_width)

        # print("mat_remover_result.projected_areas[grain_id]", mat_remover_result.projected_areas[grain_id])
        # print("grain_speed", grain_speed)
        # print("contact_length", contact_length)
        # print("grain_cutting_width", grain_cutting_width)
        # print("heat_flux", heat_flux)

    else:
        heat_flux = 0
    return heat_flux  # W/m^2


def calculate_grain_thermal_diffusivity(wp_mat_properties):
    return wp_mat_properties.thermal_conductivity / (
            wp_mat_properties.density * wp_mat_properties.specific_heat_capacity)  # m^2/s


def grain_temperature_model(mat_remover_result: MaterialRemovalResult,
                            wp_mat_properties: WorkpieceMaterialProperties,
                            specific_cutting_force,
                            simulation_time_step) -> GrainTemperatureModelResult:
    """Takazawa, Kato & Fuji Grain Grinding Temperature Calculation.
    lc contact length between workpiece and wheel lc = √a · d𝑠 or the grains displacement in each simulation step [m]
    bw workpiece width
    q𝑓 heat flux q = (Fc · vc)/(lc · bw) heat generated per unit area [W/m^2]
    vw grain_speed [m/s]
    k𝑇 thermal conductivity of the workpiece [W/(m .K)]
    α workpiece thermal diffusivity α = kT / (ρ · cp) where ρ workpiece density [m^2/s]
    Rw energy partition coefficient, unitless
    temperature_rise = 3.1 * ((2 * Rw * q𝑓 * α) / (np.pi * k𝑇 * vw)) * (((vw * lc * 0.5) / (2 * α)) ** 0.53) [K]
    """
    result = GrainTemperatureModelResult([])
    for grain_id, displacement_vector in enumerate(mat_remover_result.grains_displacement):
        contact_length = 0.01# mat_remover_result.grains_displacement[grain_id].norm() * 0.001  # [m]
        # calculate grain_velocity [N]
        grain_speed = 0.001 * mat_remover_result.grains_displacement[grain_id].norm() / simulation_time_step  # [m/s]

        heat_flux = calculate_grain_heat_flux(grain_id,
                                              mat_remover_result,
                                              specific_cutting_force,
                                              contact_length,
                                              grain_speed)  # --> heatflux [W/m^2]
        thermal_diffusivity = calculate_grain_thermal_diffusivity(wp_mat_properties)  # [m^2/s]

        temperature_rise_above_ambient_temperature = \
            3.1 * ((2 * wp_mat_properties.energy_partition_coefficient * heat_flux * thermal_diffusivity) /
                   (np.pi * wp_mat_properties.thermal_conductivity * grain_speed)) * (((grain_speed * contact_length * 0.5) / (2 * thermal_diffusivity)) ** 0.53)

        # if temperature_rise_above_ambient_temperature != 0:
        #   print("energy_partition_coefficient", wp_mat_properties.energy_partition_coefficient)
        #   print("heat flux", heat_flux)
        #   print("thermal_diffusivity", thermal_diffusivity)
        #   print("temperature_rise_above_ambient_temperature", temperature_rise_above_ambient_temperature)

        result.grains_temperature_rise.append(temperature_rise_above_ambient_temperature)

        # beta_coefficient = 0.576 * (thermal_diffusivity ** (-0.63)) * (grain_speed ** (-0.63)) * \
        #                    (contact_length ** -0.37)
        # maximum_temperature_rise_ar_workpiece_depth = temperature_rise_above_ambient_temperature *\
        #                                               np.exp(-beta_coefficient * depth)
    return result
