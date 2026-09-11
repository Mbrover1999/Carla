from scenarios.obstacle_ahead import ObstacleAheadScenario


class ScenarioRuntime:
    def __init__(self, actors=None, scenario=None):
        self.actors = list(actors or [])
        self.scenario = scenario

    def update(self, elapsed_seconds):
        callback = getattr(self.scenario, "update", None)

        if callable(callback):
            callback(elapsed_seconds)

    def apply_requested_control(self, elapsed_seconds, control):
        callback = getattr(
            self.scenario,
            "apply_requested_control",
            None
        )

        if callable(callback):
            return callback(elapsed_seconds, control)

        return control

    def controller_inactive(self, elapsed_seconds):
        callback = getattr(
            self.scenario,
            "controller_inactive",
            None
        )
        return bool(callback(elapsed_seconds)) if callable(callback) else False

    def suppress_lane_keeping(self, elapsed_seconds):
        callback = getattr(
            self.scenario,
            "suppress_lane_keeping",
            None
        )
        return bool(callback(elapsed_seconds)) if callable(callback) else False

    def force_cross_traffic_detection(self, elapsed_seconds):
        callback = getattr(
            self.scenario,
            "force_cross_traffic_detection",
            None
        )
        return bool(callback(elapsed_seconds)) if callable(callback) else False

    def notify_lane_invasion(self, detected):
        callback = getattr(
            self.scenario,
            "notify_lane_invasion",
            None
        )

        if callable(callback):
            callback(detected)

    def close(self):
        callback = getattr(self.scenario, "close", None)

        if callable(callback):
            callback()


def _runtime_for(scenario, world, ego_vehicle):
    return ScenarioRuntime(
        actors=scenario.setup(world, ego_vehicle),
        scenario=scenario
    )


def setup_scenario(scenario_id, world, ego_vehicle):
    if scenario_id == "free_drive":
        print("SCENARIO_EVENT: Free Drive initialized", flush=True)
        return ScenarioRuntime()

    if scenario_id == "obstacle_ahead":
        scenario = ObstacleAheadScenario()
        return _runtime_for(scenario, world, ego_vehicle)

    if scenario_id == "lead_vehicle_emergency_brake":
        from scenarios.lead_vehicle_emergency_brake import (
            LeadVehicleEmergencyBrakeScenario
        )

        scenario = LeadVehicleEmergencyBrakeScenario()
        return _runtime_for(scenario, world, ego_vehicle)

    if scenario_id == "cross_traffic":
        from scenarios.cross_traffic import CrossTrafficScenario

        return _runtime_for(
            CrossTrafficScenario(),
            world,
            ego_vehicle
        )

    if scenario_id == "lane_departure":
        from scenarios.lane_departure import LaneDepartureScenario

        return _runtime_for(
            LaneDepartureScenario(),
            world,
            ego_vehicle
        )

    if scenario_id == "red_traffic_light":
        from scenarios.red_traffic_light import RedTrafficLightScenario

        return _runtime_for(
            RedTrafficLightScenario(),
            world,
            ego_vehicle
        )

    if scenario_id == "driver_inactivity":
        from scenarios.driver_inactivity import DriverInactivityScenario

        return _runtime_for(
            DriverInactivityScenario(),
            world,
            ego_vehicle
        )

    raise ValueError(f"No setup is available for scenario: {scenario_id}")
