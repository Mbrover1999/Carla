from scenarios.obstacle_ahead import ObstacleAheadScenario


class LeadVehicleEmergencyBrakeScenario(ObstacleAheadScenario):
    DISTANCE_CANDIDATES_METERS = (50.0, 55.0, 60.0)
    FOLLOW_DISTANCE_METERS = 16.0
    BRAKE_AFTER_SECONDS = 4.0

    def __init__(self):
        self.lead_vehicle = None
        self.emergency_braking_started = False

    def setup(self, world, ego_vehicle):
        world_map = world.get_map()
        ego_waypoint = self._prepare_demo_location(
            world,
            world_map,
            ego_vehicle
        )

        if ego_waypoint is None:
            ego_waypoint = world_map.get_waypoint(
                ego_vehicle.get_location(),
                project_to_road=True
            )

        if ego_waypoint is None:
            raise RuntimeError(
                "Emergency Brake could not locate the ego vehicle lane"
            )

        blueprint = self._select_blueprint(
            world.get_blueprint_library()
        )

        waypoint = self._straight_waypoint_ahead(
            ego_waypoint,
            self.FOLLOW_DISTANCE_METERS
        )

        if waypoint is not None:
            lead_vehicle = world.try_spawn_actor(
                blueprint,
                self._spawn_transform(waypoint)
            )

        else:
            lead_vehicle = None

        if lead_vehicle is not None:
            lead_vehicle.set_autopilot(True)
            self.lead_vehicle = lead_vehicle
            print(
                "SCENARIO_EVENT: Lead vehicle started driving "
                f"{self.FOLLOW_DISTANCE_METERS:.0f} m ahead",
                flush=True
            )
            return [lead_vehicle]

        raise RuntimeError(
            "Emergency Brake could not place the lead vehicle"
        )

    def update(self, elapsed_seconds):
        if (
            self.lead_vehicle is None
            or self.emergency_braking_started
            or elapsed_seconds < self.BRAKE_AFTER_SECONDS
        ):
            return

        self.lead_vehicle.set_autopilot(False)
        control = self.lead_vehicle.get_control()
        control.throttle = 0.0
        control.steer = 0.0
        control.brake = 1.0
        control.hand_brake = False
        self.lead_vehicle.apply_control(control)
        self.emergency_braking_started = True
        print(
            "SCENARIO_EVENT: Lead vehicle initiated emergency braking",
            flush=True
        )
