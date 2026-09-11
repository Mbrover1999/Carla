from scenarios.obstacle_ahead import ObstacleAheadScenario
from scenarios.red_traffic_light import RedTrafficLightScenario


class CrossTrafficScenario(ObstacleAheadScenario):
    EGO_APPROACH_DISTANCE_METERS = 17.0
    CROSS_APPROACH_DISTANCE_METERS = 5.0
    CROSSING_START_SECONDS = 2.0
    CROSSING_STOP_SECONDS = 7.0
    CROSSING_THROTTLE = 0.65

    def __init__(self):
        self.crossing_vehicle = None
        self.crossing_started = False
        self.crossing_stopped = False
        self.traffic_light = None
        self.original_light_state = None

    def setup(self, world, ego_vehicle):
        blueprint = self._select_blueprint(
            world.get_blueprint_library()
        )

        for approaches in self._intersection_approaches(world):
            ego_light, ego_stop, cross_stop = approaches
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

        if (
            elapsed_seconds >= self.CROSSING_START_SECONDS
            and not self.crossing_started
        ):
            control = self.crossing_vehicle.get_control()
            control.throttle = self.CROSSING_THROTTLE
            control.steer = 0.0
            control.brake = 0.0
            control.hand_brake = False
            self.crossing_vehicle.apply_control(control)
            self.crossing_started = True
            print(
                "SCENARIO_EVENT: Cross-traffic vehicle entered the junction",
                flush=True
            )

        if (
            elapsed_seconds >= self.CROSSING_STOP_SECONDS
            and not self.crossing_stopped
        ):
            self._hold_with_brake(self.crossing_vehicle)
            self.crossing_stopped = True

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

                    if 50.0 <= angle <= 130.0:
                        yield first_light, first_stop, second_stop

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
