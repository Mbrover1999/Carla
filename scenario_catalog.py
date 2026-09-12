from dataclasses import dataclass

from config import NUMBER_OF_TRAFFIC_VEHICLES


MIN_DURATION_MINUTES = 1
MAX_DURATION_MINUTES = 60
DEFAULT_DURATION_MINUTES = 5


@dataclass(frozen=True)
class TrafficPreset:
    preset_id: str
    title: str
    vehicle_count: int

    @property
    def display_name(self):
        return f"{self.title} ({self.vehicle_count} vehicles)"


TRAFFIC_PRESETS = (
    TrafficPreset("empty", "Empty Road", 0),
    TrafficPreset("light", "Light Traffic", 10),
    TrafficPreset("moderate", "Moderate Traffic", 20),
    TrafficPreset("full", "Full Traffic", 30),
    TrafficPreset("heavy", "Heavy Traffic", 40)
)
TRAFFIC_VEHICLE_COUNTS = tuple(
    preset.vehicle_count for preset in TRAFFIC_PRESETS
)
DEFAULT_TRAFFIC_VEHICLES = (
    NUMBER_OF_TRAFFIC_VEHICLES
    if NUMBER_OF_TRAFFIC_VEHICLES in TRAFFIC_VEHICLE_COUNTS
    else 20
)


@dataclass(frozen=True)
class ScenarioDefinition:
    scenario_id: str
    title: str
    description: str
    implemented: bool = False
    demo_duration_minutes: int = 1


@dataclass(frozen=True)
class SimulationSettings:
    scenario_id: str
    duration_minutes: int = DEFAULT_DURATION_MINUTES
    traffic_vehicles: int = DEFAULT_TRAFFIC_VEHICLES

    @property
    def duration_seconds(self):
        return self.duration_minutes * 60

    def validate(self):
        scenario = get_scenario(self.scenario_id)

        if not scenario.implemented:
            raise ValueError(
                f"Scenario '{scenario.title}' is not implemented yet"
            )

        if not (
            MIN_DURATION_MINUTES
            <= self.duration_minutes
            <= MAX_DURATION_MINUTES
        ):
            raise ValueError(
                "Simulation duration must be between "
                f"{MIN_DURATION_MINUTES} and "
                f"{MAX_DURATION_MINUTES} minutes"
            )

        if self.traffic_vehicles not in TRAFFIC_VEHICLE_COUNTS:
            raise ValueError(
                "Traffic vehicles must use one of the available presets: "
                + ", ".join(
                    str(vehicle_count)
                    for vehicle_count in TRAFFIC_VEHICLE_COUNTS
                )
            )

        return self


SCENARIOS = (
    ScenarioDefinition(
        scenario_id="free_drive",
        title="Free Drive",
        description=(
            "Let the autonomous vehicle drive freely while all safety "
            "systems monitor the journey."
        ),
        implemented=True
    ),
    ScenarioDefinition(
        scenario_id="obstacle_ahead",
        title="Obstacle Ahead",
        description=(
            "Demonstrate slowing, braking and emergency braking for a "
            "vehicle or obstacle ahead."
        ),
        implemented=True
    ),
    ScenarioDefinition(
        scenario_id="lead_vehicle_emergency_brake",
        title="Lead Vehicle Emergency Brake",
        description=(
            "Follow a moving vehicle that suddenly performs a full "
            "emergency stop."
        ),
        implemented=True
    ),
    ScenarioDefinition(
        scenario_id="vehicle_cut_in",
        title="Vehicle Cut-In",
        description=(
            "A vehicle merges from the left lane into the ego lane and "
            "the safety system restores a safe following distance."
        ),
        implemented=True
    ),
    ScenarioDefinition(
        scenario_id="cross_traffic",
        title="Cross Traffic",
        description=(
            "Approach an intersection while another vehicle crosses the "
            "planned path."
        ),
        implemented=True
    ),
    ScenarioDefinition(
        scenario_id="lane_departure",
        title="Lane Departure",
        description=(
            "Trigger a controlled lane departure and demonstrate the lane "
            "keeping response."
        ),
        implemented=True
    ),
    ScenarioDefinition(
        scenario_id="red_traffic_light",
        title="Red Traffic Light",
        description=(
            "Approach a red traffic light and stop naturally near the stop "
            "line."
        ),
        implemented=True
    ),
    ScenarioDefinition(
        scenario_id="driver_inactivity",
        title="Driver Inactivity",
        description=(
            "Simulate an unresponsive driver, move right, stop on the "
            "shoulder and start a simulated emergency call."
        ),
        implemented=True,
        demo_duration_minutes=3
    )
)


def get_scenario(scenario_id):
    for scenario in SCENARIOS:
        if scenario.scenario_id == scenario_id:
            return scenario

    raise ValueError(f"Unknown scenario: {scenario_id}")


def get_traffic_preset_by_count(vehicle_count):
    for preset in TRAFFIC_PRESETS:
        if preset.vehicle_count == vehicle_count:
            return preset

    raise ValueError(f"Unknown traffic preset count: {vehicle_count}")


def get_traffic_preset_by_display_name(display_name):
    for preset in TRAFFIC_PRESETS:
        if preset.display_name == display_name:
            return preset

    raise ValueError(f"Unknown traffic preset: {display_name}")
