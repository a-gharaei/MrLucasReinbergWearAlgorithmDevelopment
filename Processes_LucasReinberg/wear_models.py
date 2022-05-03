from SimulationToolbox.PhysicalObjects.tool import *
from SimulationToolbox.Geometry.geometry import *
from SimulationToolbox.PhysicalObjects.grain import *
from SimulationToolbox.Simulation.material_removal import *
from SimulationToolbox.Simulation.wear_models import *
from typing import *


# Example wear model for wear model core tests
def wear_magnitude_as_penetration_depth_percentage(penetration_depth: float, percentage: float = 5) -> float:
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
