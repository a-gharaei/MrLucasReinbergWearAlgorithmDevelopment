class WorkpieceMaterialProperties:
    energy_partition_coefficient: float  # unitless
    thermal_conductivity: float  # [W/mK]
    density: float  # [Kg/m3]
    specific_heat_capacity: float  # [J/KgK]
    melting_point: float  # [K]

    def __init__(self,
                 energy_partition_coefficient: float,
                 thermal_conductivity: float,
                 density: float,
                 specific_heat_capacity: float,
                 melting_point: float) -> None:
        self.energy_partition_coefficient = energy_partition_coefficient  # unitless

        self.thermal_conductivity = thermal_conductivity  # W/mK
        self.density = density  # Kg/m3
        self.specific_heat_capacity = specific_heat_capacity  # J/KgK
        self.melting_point = melting_point  # [K]
