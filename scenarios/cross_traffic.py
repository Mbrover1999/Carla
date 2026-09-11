import math

from scenarios.obstacle_ahead import ObstacleAheadScenario
from scenarios.red_traffic_light import RedTrafficLightScenario


class CrossTrafficScenario(ObstacleAheadScenario):
    EGO_APPROACH_DISTANCE_METERS = 24.0
    CROSS_APPROACH_DISTANCE_METERS = 12.0
    ACTIVATION_DISTANCE_METERS = 27.0
    CROSSING_TIMEOUT_SECONDS = 12.0
    MIN_CROSSING_SPEED_MPS = 2.8
    MAX_CROSSING_SPEED_MPS = 9.0
    CROSSING_THROTTLE = 0.65

    def __init__(self):
        self.crossing_vehicle = None
        self.crossing_started = False
        self.crossing_stopped = False
        self.traffic_light = None
        self.original_light_state = None
        self.ego_vehicle = None
        self.conflict_location = None
        self.crossing_direction = None

    def setup(self, world, ego_vehicle):
        blueprint = self._select_blueprint(
            world.get_blueprint_library()
        )

        for approaches in self._intersection_approaches(world):
            ego_light, ego_stop, cross_stop, conflict_location = approaches
            ego_waypoints = ego_stop.previous(
                self.EGO_APPROACH_DISTANCE_METERS
            )
            cross_waypoints = cross_stop.previous(
                self.CROSS_APPROACH_DISTANCE_METERS
            )

            for ego_waypoint in ego_waypoints:
                if getattr(ego_waypoint, "is_junction", False):
                    continue

                for cross_waypoint in cross_waypoints:
                    if getattr(cross_waypoint, "is_junction", False):
                        continue

                    crossing_vehicle = world.try_spawn_actor(
                        blueprint,
                        self._spawn_transform(cross_waypoint)
                    )

                    if crossing_vehicle is None:
                        continue

                    try:
                        crossing_vehicle.set_autopilot(False)
                        self._hold_with_brake(crossing_vehicle)
                        ego_vehicle.set_transform(
                            self._spawn_transform(ego_waypoint)
                        )
                        self._stop_ego_vehicle(ego_vehicle)
                        self._force_green(ego_light)
                        world.wait_for_tick()
                    except (AttributeError, RuntimeError):
                        try:
                            crossing_vehicle.destroy()
                        except (AttributeError, RuntimeError):
                            pass
                        self.close()
                        continue

                    self.crossing_vehicle = crossing_vehicle
                    self.ego_vehicle = ego_vehicle
                    self.conflict_location = conflict_location
                    self.crossing_direction = self._direction(cross_waypoint)
                    print(
                        "SCENARIO_EVENT: Cross Traffic demo ready; a "
                        "vehicle will enter from the side",
                        flush=True
                    )
                    return [crossing_vehicle]

        raise RuntimeError(
            "Cross Traffic could not find a suitable intersection"
        )

    def update(self, elapsed_seconds):
        if self.crossing_vehicle is None:
            return

        if not self.crossing_started and self._should_start_crossing():
            crossing_speed = self._synchronized_crossing_speed()
            control = self.crossing_vehicle.get_control()
            control.throttle = self.CROSSING_THROTTLE
            control.steer = 0.0
            control.brake = 0.0
            control.hand_brake = False
            self.crossing_vehicle.apply_control(control)
            self._enable_constant_velocity(crossing_speed)
            self.crossing_started = True
            print(
                "SCENARIO_EVENT: Cross-traffic vehicle entered the "
                f"collision path at {crossing_speed:.1f} m/s",
                flush=True
            )

        if (
            self.crossing_started
            and not self.crossing_stopped
            and self._distance_to_conflict(self.crossing_vehicle) > 8.0
        ):
            self._enable_constant_velocity(
                self._synchronized_crossing_speed()
            )

        if (
            self.crossing_started
            and (
                elapsed_seconds >= self.CROSSING_TIMEOUT_SECONDS
                or self._crossing_vehicle_cleared_conflict()
            )
            and not self.crossing_stopped
        ):
            self._disable_constant_velocity()
            self._hold_with_brake(self.crossing_vehicle)
            self.crossing_stopped = True

    @staticmethod
    def force_cross_traffic_detection(_elapsed_seconds):
        return True

    def close(self):
        if self.traffic_light is None:
            return

        try:
            self.traffic_light.freeze(False)

            if self.original_light_state is not None:
                self.traffic_light.set_state(
                    self.original_light_state
                )
        except (AttributeError, RuntimeError):
            pass

        self.traffic_light = None

    def _force_green(self, traffic_light):
        original_state = traffic_light.get_state()
        green_state = RedTrafficLightScenario._named_state(
            original_state,
            "Green"
        )

        if green_state is None:
            raise RuntimeError("Could not resolve CARLA green-light state")

        self.traffic_light = traffic_light
        self.original_light_state = original_state
        traffic_light.set_state(green_state)
        traffic_light.freeze(True)

    @classmethod
    def _intersection_approaches(cls, world):
        seen_groups = set()

        for seed_light in RedTrafficLightScenario._traffic_lights(world):
            try:
                group = list(seed_light.get_group_traffic_lights())
            except (AttributeError, RuntimeError):
                group = [seed_light]

            group_key = tuple(sorted(
                getattr(light, "id", id(light))
                for light in group
            ))

            if group_key in seen_groups:
                continue

            seen_groups.add(group_key)
            stops = []

            for light in group:
                try:
                    stops.extend(
                        (light, waypoint)
                        for waypoint in light.get_stop_waypoints()
                    )
                except (AttributeError, RuntimeError):
                    continue

            for first_index, (first_light, first_stop) in enumerate(stops):
                for _, second_stop in stops[first_index + 1:]:
                    angle = cls._heading_difference(
                        first_stop,
                        second_stop
                    )

                    conflict = cls._line_conflict(
                        first_stop,
                        second_stop
                    )

                    if (
                        50.0 <= angle <= 130.0
                        and conflict is not None
                    ):
                        yield (
                            first_light,
                            first_stop,
                            second_stop,
                            conflict
                        )

    def _should_start_crossing(self):
        if self.ego_vehicle is None or self.conflict_location is None:
            return False

        velocity = self.ego_vehicle.get_velocity()
        ego_speed = math.hypot(velocity.x, velocity.y)
        return (
            ego_speed >= 2.5
            and self._distance_to_conflict(self.ego_vehicle)
            <= self.ACTIVATION_DISTANCE_METERS
        )

    def _synchronized_crossing_speed(self):
        ego_distance = self._distance_to_conflict(self.ego_vehicle)
        crossing_distance = self._distance_to_conflict(
            self.crossing_vehicle
        )
        ego_velocity = self.ego_vehicle.get_velocity()
        ego_speed = max(
            math.hypot(ego_velocity.x, ego_velocity.y),
            2.5
        )
        ego_arrival_seconds = max(1.0, ego_distance / ego_speed)
        requested_speed = crossing_distance / ego_arrival_seconds
        return self._clip(
            requested_speed,
            self.MIN_CROSSING_SPEED_MPS,
            self.MAX_CROSSING_SPEED_MPS
        )

    def _enable_constant_velocity(self, speed_mps):
        try:
            velocity = self.crossing_vehicle.get_velocity()
            velocity.x = self.crossing_direction[0] * speed_mps
            velocity.y = self.crossing_direction[1] * speed_mps
            velocity.z = 0.0
            self.crossing_vehicle.enable_constant_velocity(velocity)
        except (AttributeError, RuntimeError):
            pass

    def _disable_constant_velocity(self):
        try:
            self.crossing_vehicle.disable_constant_velocity()
        except (AttributeError, RuntimeError):
            pass

    def _crossing_vehicle_cleared_conflict(self):
        if self._distance_to_conflict(self.crossing_vehicle) > 12.0:
            actor_location = self.crossing_vehicle.get_location()
            offset_x = actor_location.x - self.conflict_location[0]
            offset_y = actor_location.y - self.conflict_location[1]
            return (
                offset_x * self.crossing_direction[0]
                + offset_y * self.crossing_direction[1]
            ) > 0.0

        return False

    def _distance_to_conflict(self, vehicle):
        location = vehicle.get_location()
        return math.hypot(
            location.x - self.conflict_location[0],
            location.y - self.conflict_location[1]
        )

    @classmethod
    def _line_conflict(cls, first_waypoint, second_waypoint):
        first_location = first_waypoint.transform.location
        second_location = second_waypoint.transform.location
        first_direction = cls._direction(first_waypoint)
        second_direction = cls._direction(second_waypoint)
        denominator = cls._cross(first_direction, second_direction)

        if abs(denominator) < 0.01:
            return None

        offset = (
            second_location.x - first_location.x,
            second_location.y - first_location.y
        )
        first_distance = cls._cross(offset, second_direction) / denominator
        second_distance = cls._cross(offset, first_direction) / denominator

        if not (0.0 <= first_distance <= 25.0):
            return None

        if not (0.0 <= second_distance <= 25.0):
            return None

        return (
            first_location.x + first_direction[0] * first_distance,
            first_location.y + first_direction[1] * first_distance
        )

    @staticmethod
    def _direction(waypoint):
        yaw = math.radians(waypoint.transform.rotation.yaw)
        return math.cos(yaw), math.sin(yaw)

    @staticmethod
    def _cross(first, second):
        return first[0] * second[1] - first[1] * second[0]

    @staticmethod
    def _clip(value, minimum, maximum):
        return max(minimum, min(value, maximum))

    @staticmethod
    def _heading_difference(first_waypoint, second_waypoint):
        first_yaw = first_waypoint.transform.rotation.yaw
        second_yaw = second_waypoint.transform.rotation.yaw
        difference = (second_yaw - first_yaw + 180.0) % 360.0 - 180.0
        return abs(difference)

    @staticmethod
    def _hold_with_brake(vehicle):
        control = vehicle.get_control()
        control.throttle = 0.0
        control.steer = 0.0
        control.brake = 1.0
        control.hand_brake = False
        vehicle.apply_control(control)
