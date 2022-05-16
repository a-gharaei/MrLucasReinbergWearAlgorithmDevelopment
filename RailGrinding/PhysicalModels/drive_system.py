import math
import yaml


class DriveSystemResullt:
    spindle_current: float
    mechanical_power_of_the_spindle: float
    electrical_power_of_the_spindle: float

    def __init__(self, spindle_current, mechanical_power_of_the_spindle, electrical_power_of_the_spindle):
        self.spindle_current = spindle_current
        self.mechanical_power_of_the_spindle = mechanical_power_of_the_spindle
        self.electrical_power_of_the_spindle = electrical_power_of_the_spindle


# drive system
# definition:
class DriveSystemConfiguration:
    motor_efficiency: float  # efficiency of the electric motor
    spindle_voltage: float  # [V] nominal voltage of the spindle
    slip_factor: float  # slip of the asynchronous motor

    surface_lower_chamber: float  # [N/bar] surface of the lower chamber of the pneumatic cylinder
    surface_upper_chamber: float  # [N/bar] surface of the upper chamber of the pneumatic cylinder
    pressure_upper_chamber: float  # [bar] pressure of the upper chamber of the pneumatic cylinder
    mass_force: float  # [N] mass force of the spindle

    def __init__(self, motor_efficiency: float, spindle_voltage: float, slip_factor: float,
                 surface_lower_chamber: float, surface_upper_chamber: float, pressure_upper_chamber: float,
                 mass_force: float) -> None:
        self.motor_efficiency = motor_efficiency
        self.spindle_voltage = spindle_voltage
        self.slip_factor = slip_factor

        self.surface_lower_chamber = surface_lower_chamber
        self.surface_upper_chamber = surface_upper_chamber
        self.pressure_upper_chamber = pressure_upper_chamber
        self.mass_force = mass_force

    @classmethod
    def from_config_file(cls, path):
        with open(path) as file:
            configs = yaml.full_load(file)
            dict = configs['drive_system']
        return cls(dict['motor_efficiency'],
                   dict['spindle_voltage'],
                   dict['slip_factor'],

                   dict['surface_lower_chamber'],
                   dict['surface_upper_chamber'],
                   dict['pressure_upper_chamber'],
                   dict['mass_force'])

    def save_configuration_in_yaml(self):
        # TODO: we may need to add this functionality or we may not!
        pass


# calculation of the actual spindle current and electrical and  mechanical power
def calculate_spindle_signals(tangential_force, rotational_wheel_speed, wheel_average_radius, config):
    mechanical_power_of_the_spindle = abs(
        tangential_force) * wheel_average_radius * rotational_wheel_speed  # [W] tangential force and linear velocity
    # of the grinding wheel
    electrical_power_of_the_spindle = mechanical_power_of_the_spindle / config.motor_efficiency  # [W]
    spindle_current = electrical_power_of_the_spindle / (
            math.sqrt(3) * config.spindle_voltage * config.slip_factor)  # [A]
    drive_system_results = DriveSystemResullt(spindle_current, mechanical_power_of_the_spindle,
                                              electrical_power_of_the_spindle)
    return drive_system_results
