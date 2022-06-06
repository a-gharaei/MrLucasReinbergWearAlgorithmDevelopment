import math
from typing import *

import trimesh
from SimulationToolbox.Geometry.geometry import *
from SimulationToolbox.PhysicalObjects.grain import *
from SimulationToolbox.PhysicalObjects.tool import *
from SimulationToolbox.Simulation.material_removal import *
from SimulationToolbox.Simulation.physical_models import *
from SimulationToolbox.Simulation.wear_models import *

from FractureWear.wear_algorithm import macro_fracture


# Example wear model for wear model core tests
def wear_magnitude_as_penetration_depth_percentage(
    penetration_depth: float, percentage: float = 5
) -> float:
    return penetration_depth * percentage / 100


def apply_attritious_wear_model(
    tool: Tool,
    wear_magnitude_model: Callable[[float], float],
    mat_remover_result: MaterialRemovalResult,
) -> AttritiousWearModelResult:

    wears_vector = []
    wears_magnitude = []
    for grain_idx, grain in enumerate(tool.grains):

        if mat_remover_result.penetration_depths[grain_idx] == 0:
            wears_vector.append(Vector.origin())
            wears_magnitude.append(0)
            continue

        wear_magnitude = wear_magnitude_model(
            mat_remover_result.penetration_depths[grain_idx]
        )
        wears_magnitude.append(wear_magnitude)
        wears_vector.append(
            mat_remover_result.wear_directions[grain_idx]
            .normalize()
            .scale(wear_magnitude)
        )

    return AttritiousWearModelResult(wears_vector, wears_magnitude)


def apply_fracture_wear_model(
    tool: Tool,
    mat_remover_result: MaterialRemovalResult,
    grain_force_result: GrainForceModelResult,
):
    rankine_stresses = []
    penetration_depths = []
    fractured_grains = []
    fracture_informations = []
    total_removed_volume = 0
    for grain_idx, grain in enumerate(tool.grains):
        # Error handling
        if math.isnan(mat_remover_result.penetration_depths[grain_idx]):
            print("penetration dpeth is nan")
            print(f"grain position: {tool.grains[0].get_position().value}")
            continue
        if mat_remover_result.penetration_depths[grain_idx] == 0:
            rankine_stresses.append(0)
            penetration_depths.append(0)
            continue
        # Getting the inputs from iBrus
        penetration_depth = mat_remover_result.penetration_depths[grain_idx]
        cutting_force = grain_force_result.grain_forces[grain_idx].cutting_force.norm()
        normal_force = grain_force_result.grain_forces[grain_idx].normal_force.norm()
        cutting_direction = mat_remover_result.grains_displacement[
            grain_idx
        ].change_frame_vector([], [tool.pose, tool.grains[grain_idx].pose])
        # Macri fracture wear algorithm
        removed_volume, rankine = macro_fracture(
            grain,
            cutting_force,
            normal_force,
            penetration_depth,
            cutting_direction,
            300,
        )
        # Documentation of results
        if rankine > 300:
            print("fracture occured")
            fractured_grains.append(tool.grains[grain_idx].get_mesh())
            fracture_informations.append(
                [rankine, penetration_depth, cutting_force, normal_force]
            )
        total_removed_volume += removed_volume
        rankine_stresses.append(rankine)
        penetration_depths.append(penetration_depth)
    return (
        total_removed_volume,
        rankine_stresses,
        penetration_depths,
        fractured_grains,
        fracture_informations,
    )
