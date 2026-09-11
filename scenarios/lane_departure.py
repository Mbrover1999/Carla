from scenarios.obstacle_ahead import ObstacleAheadScenario


class LaneDepartureScenario(ObstacleAheadScenario):
    DISTANCE_CANDIDATES_METERS = (60.0, 70.0, 80.0)
    DEPARTURE_START_SECONDS = 4.0
    DEPARTURE_END_SECONDS = 5.5
    DEPARTURE_STEERING = 0.24

    def __init__(self):
        self.departure_announced = False
        self.recovery_announced = False

    def setup(self, world, ego_vehicle):
        waypoint = self._prepare_demo_location(
            world,
            world.get_map(),
            ego_vehicle
        )

        if waypoint is None:
            raise RuntimeError(
                "Lane Departure could not find a long straight road"
            )

        print(
            "SCENARIO_EVENT: Lane Departure demo ready; vehicle will "
            "drift right after 4 seconds",
            flush=True
        )
        return []

    def update(self, elapsed_seconds):
        if (
            elapsed_seconds >= self.DEPARTURE_START_SECONDS
            and not self.departure_announced
        ):
            self.departure_announced = True
            print(
                "SCENARIO_EVENT: Controlled right lane departure started",
                flush=True
            )

        if (
            elapsed_seconds >= self.DEPARTURE_END_SECONDS
            and not self.recovery_announced
        ):
            self.recovery_announced = True
            print(
                "SCENARIO_EVENT: Lane keeping released to recover the vehicle",
                flush=True
            )

    def apply_requested_control(self, elapsed_seconds, control):
        if not self._departure_active(elapsed_seconds):
            return control

        control_type = type(control)
        return control_type(
            throttle=control.throttle,
            steer=self.DEPARTURE_STEERING,
            brake=control.brake,
            hand_brake=getattr(control, "hand_brake", False),
            reverse=getattr(control, "reverse", False),
            manual_gear_shift=getattr(
                control,
                "manual_gear_shift",
                False
            ),
            gear=getattr(control, "gear", 0)
        )

    def suppress_lane_keeping(self, elapsed_seconds):
        return self._departure_active(elapsed_seconds)

    def _departure_active(self, elapsed_seconds):
        return (
            self.DEPARTURE_START_SECONDS
            <= elapsed_seconds
            < self.DEPARTURE_END_SECONDS
        )
