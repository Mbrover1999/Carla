from scenarios.obstacle_ahead import ObstacleAheadScenario


class DriverInactivityScenario(ObstacleAheadScenario):
    DISTANCE_CANDIDATES_METERS = (80.0, 90.0, 100.0)
    INACTIVITY_START_SECONDS = 4.0

    def __init__(self):
        self.inactivity_announced = False

    def setup(self, world, ego_vehicle):
        waypoint = self._prepare_demo_location(
            world,
            world.get_map(),
            ego_vehicle
        )

        if waypoint is None:
            raise RuntimeError(
                "Driver Inactivity could not find a suitable straight road"
            )

        print(
            "SCENARIO_EVENT: Driver Inactivity demo ready; controller "
            "response will stop after 4 seconds",
            flush=True
        )
        return []

    def update(self, elapsed_seconds):
        if (
            elapsed_seconds >= self.INACTIVITY_START_SECONDS
            and not self.inactivity_announced
        ):
            self.inactivity_announced = True
            print(
                "SCENARIO_EVENT: Driver/controller stopped responding",
                flush=True
            )

    def controller_inactive(self, elapsed_seconds):
        return elapsed_seconds >= self.INACTIVITY_START_SECONDS

    @staticmethod
    def _is_demo_waypoint_suitable(waypoint):
        try:
            right_lane = waypoint.get_right_lane()
        except (AttributeError, RuntimeError):
            return True

        if right_lane is None:
            return False

        lane_type = getattr(
            getattr(right_lane, "lane_type", None),
            "name",
            str(getattr(right_lane, "lane_type", ""))
        ).upper()
        return lane_type in {"DRIVING", "SHOULDER", "PARKING"}
