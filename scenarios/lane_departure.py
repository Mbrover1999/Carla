from scenarios.obstacle_ahead import ObstacleAheadScenario


class LaneDepartureScenario(ObstacleAheadScenario):
    DISTANCE_CANDIDATES_METERS = (60.0, 70.0, 80.0)
    DEPARTURE_START_SECONDS = 4.0
    DEPARTURE_END_SECONDS = 6.0
    TARGET_LATERAL_OFFSET_METERS = 1.15
    MIN_DEPARTURE_STEERING = 0.035
    MAX_DEPARTURE_STEERING = 0.09
    STEERING_GAIN = 0.055

    def __init__(self):
        self.departure_announced = False
        self.recovery_announced = False
        self.departure_complete = False
        self.world_map = None
        self.ego_vehicle = None

    def setup(self, world, ego_vehicle):
        self.world_map = world.get_map()
        self.ego_vehicle = ego_vehicle
        waypoint = self._prepare_demo_location(
            world,
            self.world_map,
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
            (
                self.departure_complete
                or elapsed_seconds >= self.DEPARTURE_END_SECONDS
            )
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

        lateral_offset = self._lateral_offset()

        if lateral_offset is not None:
            remaining_offset = (
                self.TARGET_LATERAL_OFFSET_METERS
                - lateral_offset
            )

            if remaining_offset <= 0.0:
                self.departure_complete = True
                return control

            steering = self._clip(
                remaining_offset * self.STEERING_GAIN,
                self.MIN_DEPARTURE_STEERING,
                self.MAX_DEPARTURE_STEERING
            )
        else:
            steering = self.MIN_DEPARTURE_STEERING

        control_type = type(control)
        return control_type(
            throttle=min(control.throttle, 0.12),
            steer=steering,
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

    def notify_lane_invasion(self, detected):
        if detected:
            self.departure_complete = True

    def _departure_active(self, elapsed_seconds):
        return (
            not self.departure_complete
            and
            self.DEPARTURE_START_SECONDS
            <= elapsed_seconds
            < self.DEPARTURE_END_SECONDS
        )

    def _lateral_offset(self):
        if self.world_map is None or self.ego_vehicle is None:
            return None

        try:
            vehicle_transform = self.ego_vehicle.get_transform()
            waypoint = self.world_map.get_waypoint(
                vehicle_transform.location,
                project_to_road=True
            )
        except (AttributeError, RuntimeError):
            return None

        if waypoint is None:
            return None

        return self._signed_lateral_offset(
            vehicle_transform,
            waypoint.transform
        )

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
        same_direction = (
            getattr(waypoint, "lane_id", 0)
            * getattr(right_lane, "lane_id", 0)
            >= 0
        )
        return lane_type == "DRIVING" and same_direction

    @staticmethod
    def _signed_lateral_offset(vehicle_transform, waypoint_transform):
        import math

        dx = vehicle_transform.location.x - waypoint_transform.location.x
        dy = vehicle_transform.location.y - waypoint_transform.location.y
        yaw = math.radians(waypoint_transform.rotation.yaw)
        return dx * -math.sin(yaw) + dy * math.cos(yaw)

    @staticmethod
    def _clip(value, minimum, maximum):
        return max(minimum, min(value, maximum))
