from collections import defaultdict
from typing import List
import dill
import numpy as np
import pandas as pd

from SimulationToolbox.Geometry.geometry import PoseTrajectory
from SimulationToolbox.Simulation.physical_models import ToolForceModelResultListFloat, ToolForceModelResult

from RailGrinding.PhysicalModels.drive_system import *


def calculate_average_forces(tool_forces_array, controller_timing):
    recent_average_normal_force = np.average(tool_forces_array.tool_forces_z[-controller_timing:])
    recent_average_tangential_force = np.average(tool_forces_array.tool_forces_y[-controller_timing:])
    return recent_average_normal_force, recent_average_tangential_force


def update_tool_trajectory(new_z_position, tool_trajectory, sim_i):
    for i in range(len(tool_trajectory.poses[sim_i + 1:])):
        tool_trajectory.poses[sim_i + 1 + i].value.value[2][3] = new_z_position


class PidControllerResults:
    calculated_normal_force: float
    manip_current: float
    spindle_current: float
    current_wheel_z_position: float
    next_wheel_z_position: float

    def __init__(self,
                 calculated_normal_force: float,
                 manip_current: float,
                 spindle_current: float,
                 current_wheel_z_position: float,
                 next_wheel_z_position: float):
        self.calculated_normal_force = calculated_normal_force
        self.manip_current = manip_current
        self.spindle_current = spindle_current
        self.current_wheel_z_position = current_wheel_z_position
        self.next_wheel_z_position = next_wheel_z_position


class PidControllerResultList:
    pid_controller_results_list: List[PidControllerResults]

    def __init__(self, pid_controller_result_list: List[PidControllerResults]):
        self.pid_controller_results_list = pid_controller_result_list

    def save_to_disk(self, file_name: str):
        with open(file_name, 'wb') as file:
            dill.dump(self, file)

    @staticmethod
    def load_from_disk(file_name):
        with open(file_name, 'rb') as file:
            object = dill.load(file)
        if isinstance(object, PidControllerResultList):
            return object
        else:
            Exception(
                "The loaded PidControllerResultList object is not from the current version of this class.")

    def get_in_dataframe(self):
        dicts = []
        for i in range(len(self.pid_controller_results_list)):
            dicts.append(vars(self.pid_controller_results_list[i]))
        results_dictionary = defaultdict(list)
        for dict in dicts:
            for key, value in dict.items():
                results_dictionary[key].append(value)
        df = pd.DataFrame.from_dict(results_dictionary)
        return (df)


# implementation of the real process control loop
class PidController:
    integral = 0  # initial value of the integral part of the PID-Controller
    __parameters: np.ndarray
    nominal_current: float  # [A] nominal current of the grinding process control
    controller_timing: float
    change_in_z_position_by_controller: float

    # definition of the PID parameters
    def __init__(self, parameter_i: float, parameter_p: float, parameter_t, nominal_current: float,
                 controller_timing: float, change_in_z_position_by_controller: float):
        self.__parameters = np.array(
            [parameter_i, parameter_p, parameter_t])  # proportional & integral coefficient of the PID-Controller
        self.nominal_current = nominal_current
        self.controller_timing = controller_timing
        self.change_in_z_position_by_controller = change_in_z_position_by_controller

    # calculation of the manipulated current with help of a PID-Controller
    def calculate_pid_current(self, tangential_force, wheel_speed, p_parameter, i_parameter, t_parameter,
                              wheel_average_radius, drive_sys_config):

        spindle_signal_results = calculate_spindle_signals(tangential_force, wheel_speed, wheel_average_radius,
                                                           drive_sys_config)
        error = self.nominal_current - spindle_signal_results.spindle_current
        proportional = p_parameter * error  # Proportional part of the PID controller
        integral_new = i_parameter * error * t_parameter  # Integral part of the PID controller
        self.integral += integral_new  # sum of the integral part
        manip_current = proportional + self.integral  # manipulated value of current
        # print("mech power", spindle_signal_results.mechanical_power_of_the_spindle)
        # print("error", error)
        # print("manip_current", manip_current, "spindle_current ", spindle_signal_results.spindle_current)
        return manip_current, spindle_signal_results.spindle_current

    # calculation of the adjusted normal force
    def calculate_adjusted_normal_force(self, tangential_force, wheel_speed, wheel_average_radius, drive_sys_config):
        manip_current, spindle_current = self.calculate_pid_current(tangential_force,
                                                                    wheel_speed,
                                                                    self.__parameters[1],
                                                                    self.__parameters[0],
                                                                    self.__parameters[2],
                                                                    wheel_average_radius,
                                                                    drive_sys_config)
        pressure_lower_chamber = (47 - 0.9 * manip_current) * 0.09
        adjusted_normal_force = drive_sys_config.mass_force + drive_sys_config.pressure_upper_chamber * \
                                drive_sys_config.surface_upper_chamber - pressure_lower_chamber * \
                                drive_sys_config.surface_lower_chamber  # [N]
        return adjusted_normal_force, manip_current, spindle_current

    # to calculate and return controller results
    def get_controller_results(self,
                               wheel_speed: float,
                               wheel_average_radius: float,
                               tool_force_results: List[ToolForceModelResult],
                               tool_trajectory: PoseTrajectory,
                               sim_i: int,  # current simulation step
                               controller_timing: float,
                               drive_sys_config: DriveSystemConfiguration):

        if ((sim_i + 1) % controller_timing) == 0:
            # convert to handier format
            tool_forces_array = ToolForceModelResultListFloat.from_tool_force_model_result_list(
                tool_force_results[-controller_timing:])

            # print("inja1", tool_trajectory.poses[-controller_timing:])
            # print("inja2", tool_trajectory.slice(-controller_timing).poses)
            # for release 1.2.0
            global_forces_array = tool_forces_array.get_forces_in_global(tool_trajectory.poses[-controller_timing:])

            global_forces_array = tool_forces_array.get_forces_in_global(tool_trajectory.slice(-controller_timing))
            z_position = tool_trajectory.poses[sim_i].value.value[2][3]  # current_tool_height)

            average_kienzle_tool_normal_force, average_kienzle_tool_tangential_force = \
                calculate_average_forces(global_forces_array, controller_timing)

            calculated_normal_force, manip_current, spindle_current = self.calculate_adjusted_normal_force(
                abs(average_kienzle_tool_tangential_force), wheel_speed, wheel_average_radius, drive_sys_config)
            print("calculated_normal_force", calculated_normal_force, "kienzle_normal_force",
                  abs(average_kienzle_tool_normal_force))
            print("last_tangential_force", abs(average_kienzle_tool_tangential_force))

            if calculated_normal_force < abs(average_kienzle_tool_normal_force):
                # increase the depth of the grinding wheel
                next_wheel_z_position = z_position + self.change_in_z_position_by_controller
            elif calculated_normal_force > abs(average_kienzle_tool_normal_force):
                # decrease the depth of the grinding wheel
                next_wheel_z_position = z_position - self.change_in_z_position_by_controller
            else:
                next_wheel_z_position = z_position

            print("new Z: ", next_wheel_z_position)

            results = PidControllerResults(calculated_normal_force,
                                           manip_current,
                                           spindle_current,
                                           z_position,
                                           next_wheel_z_position)
            update_tool_trajectory(results.next_wheel_z_position, tool_trajectory, sim_i)
            return results
        else:
            results = PidControllerResults(None, None, None, None, None)
            return results
