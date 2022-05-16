from collections import defaultdict
from typing import List
import dill
import numpy as np
import pandas as pd

from SimulationToolbox.Geometry.geometry import PoseTrajectory
from SimulationToolbox.Simulation.physical_models import ToolForceModelResultListFloat, ToolForceModelResult


def calculate_average_normal_force_error(tool_force_results, reference_force, tool_trajectory, controller_timing):
    # convert to handier format
    tool_forces_array = ToolForceModelResultListFloat.from_tool_force_model_result_list(
        tool_force_results[-controller_timing:])
    global_forces_array = tool_forces_array.get_forces_in_global(tool_trajectory.poses[- controller_timing:])
    current_average_normal_force = np.average(global_forces_array.tool_forces_z[-controller_timing:])
    # current_average_tangential_force = np.average(global_forces_array.tool_forces_y[-controller_timing:])
    force_error = (current_average_normal_force - reference_force) / reference_force  # unitless ratio
    return force_error, current_average_normal_force


def update_tool_trajectory(new_z_position, tool_trajectory, sim_i):
    for i in range(len(tool_trajectory.poses[sim_i + 1:])):
        tool_trajectory.poses[sim_i + 1 + i].value.value[2][3] = new_z_position


# implementation of the real process control loop
def calculate_current_and_last_tool_z_position(tool_trajectory, controller_timing, sim_i):
    current_z_position = tool_trajectory.poses[sim_i].value.value[2][3]  # current_tool_height [mm]
    last_z_position = tool_trajectory.poses[sim_i - controller_timing].value.value[2][3]  # last_tool_height [mm]
    return current_z_position, last_z_position


class ProportionalControllerResults:
    force_error: float
    current_z_position: float
    next_wheel_z_position: float

    def __init__(self,
                 force_error: float,
                 current_z_position: float,
                 next_wheel_z_position: float,
                 average_wheel_normal_force: float):
        self.force_error = force_error
        self.current_z_position = current_z_position
        self.next_wheel_z_position = next_wheel_z_position
        self.average_wheel_normal_force = average_wheel_normal_force


class ProportionalControllerResultList:
    proportional_controller_results_list: List[ProportionalControllerResults]

    def __init__(self, proportional_controller_result_list: List[ProportionalControllerResults]):
        self.proportional_controller_results_list = proportional_controller_result_list

    def save_to_disk(self, file_name: str):
        with open(file_name, 'wb') as file:
            dill.dump(self, file)

    def get_in_dataframe(self):
        dictionaries = []
        for i in range(len(self.proportional_controller_results_list)):
            dictionaries.append(vars(self.proportional_controller_results_list[i]))
        results_dictionary = defaultdict(list)
        for dictionary in dictionaries:
            for key, value in dictionary.items():
                results_dictionary[key].append(value)
        df = pd.DataFrame.from_dict(results_dictionary)
        return df

    @staticmethod
    def load_from_disk(file_name):
        with open(file_name, 'rb') as file:
            item = dill.load(file)
        if isinstance(item, ProportionalControllerResultList):
            return item
        else:
            Exception(
                "The loaded ProportionalControllerResultList object is not from the current version of this class.")


class ProportionalController:
    proportional_gain: float  #

    def __init__(self, proportional_gain: float):
        self.proportional_gain = proportional_gain

    # to calculate and return controller results
    def get_proportional_controller_results(self,
                                            tool_force_results: List[ToolForceModelResult],
                                            tool_trajectory: PoseTrajectory,
                                            sim_i: int,  # current simulation step
                                            controller_timing: float,
                                            reference_force: float,
                                            wear_compensation: float):

        if ((sim_i + 1) % controller_timing) == 0:

            wear_compensation = wear_compensation  # [mm]
            proportional_adjustment = 0
            force_error = None
            average_wheel_normal_force = None

            if (sim_i + 1) >= controller_timing:
                force_error, average_wheel_normal_force = calculate_average_normal_force_error(tool_force_results,
                                                                                               reference_force,
                                                                                               tool_trajectory,
                                                                                               controller_timing)
                proportional_adjustment = force_error * self.proportional_gain

            current_z_position = tool_trajectory.poses[sim_i].value.value[2][3]  # current_tool_height [mm]
            next_wheel_z_position = current_z_position + proportional_adjustment + wear_compensation

            results = ProportionalControllerResults(force_error, current_z_position, next_wheel_z_position,
                                                    average_wheel_normal_force)
            update_tool_trajectory(next_wheel_z_position, tool_trajectory, sim_i)
            return results
        else:
            results = ProportionalControllerResults(None, None, None, None)
        return results
