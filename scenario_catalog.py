from dataclasses import dataclass

from config import NUMBER_OF_TRAFFIC_VEHICLES


MIN_DURATION_MINUTES = 1
MAX_DURATION_MINUTES = 60
MIN_TRAFFIC_VEHICLES = 0
MAX_TRAFFIC_VEHICLES = 100
DEFAULT_DURATION_MINUTES = 5
DEFAULT_TRAFFIC_VEHICLES = NUMBER_OF_TRAFFIC_VEHICLES


@dataclass(frozen=True)
class ScenarioDefinition:
    scenario_id: str
    title: str
    description: str
    implemented: bool = False


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

        if not (
            MIN_TRAFFIC_VEHICLES
            <= self.traffic_vehicles
            <= MAX_TRAFFIC_VEHICLES
        ):
            raise ValueError(
                "Number of traffic vehicles must be between "
                f"{MIN_TRAFFIC_VEHICLES} and "
                f"{MAX_TRAFFIC_VEHICLES}"
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
        )
    ),
    ScenarioDefinition(
        scenario_id="cross_traffic",
        title="Cross Traffic",
        description=(
            "Approach an intersection while another vehicle crosses the "
            "planned path."
        )
    ),
    ScenarioDefinition(
        scenario_id="lane_departure",
        title="Lane Departure",
        description=(
            "Trigger a controlled lane departure and demonstrate the lane "
            "keeping response."
        )
    ),
    ScenarioDefinition(
        scenario_id="red_traffic_light",
        title="Red Traffic Light",
        description=(
            "Approach a red traffic light and stop naturally near the stop "
            "line."
        )
    ),
    ScenarioDefinition(
        scenario_id="driver_inactivity",
        title="Driver Inactivity",
        description=(
            "Simulate an unresponsive driver, move right, stop on the "
            "shoulder and start a simulated emergency call."
        )
    )
)


def get_scenario(scenario_id):
    for scenario in SCENARIOS:
        if scenario.scenario_id == scenario_id:
            return scenario

    raise ValueError(f"Unknown scenario: {scenario_id}")
