from scenarios.obstacle_ahead import ObstacleAheadScenario


class ScenarioRuntime:
    def __init__(self, actors=None, update_callback=None):
        self.actors = list(actors or [])
        self.update_callback = update_callback

    def update(self, elapsed_seconds):
        if self.update_callback is not None:
            self.update_callback(elapsed_seconds)


def setup_scenario(scenario_id, world, ego_vehicle):
    if scenario_id == "free_drive":
        print("SCENARIO_EVENT: Free Drive initialized", flush=True)
        return ScenarioRuntime()

    if scenario_id == "obstacle_ahead":
        scenario = ObstacleAheadScenario()
        return ScenarioRuntime(
            actors=scenario.setup(world, ego_vehicle)
        )

    if scenario_id == "lead_vehicle_emergency_brake":
        from scenarios.lead_vehicle_emergency_brake import (
            LeadVehicleEmergencyBrakeScenario
        )

        scenario = LeadVehicleEmergencyBrakeScenario()
        return ScenarioRuntime(
            actors=scenario.setup(world, ego_vehicle),
            update_callback=scenario.update
        )

    raise ValueError(f"No setup is available for scenario: {scenario_id}")
