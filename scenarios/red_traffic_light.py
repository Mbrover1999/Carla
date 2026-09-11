class RedTrafficLightScenario:
    APPROACH_DISTANCE_METERS = 28.0

    def __init__(self):
        self.traffic_light = None
        self.original_state = None

    def setup(self, world, ego_vehicle):
        for traffic_light in self._traffic_lights(world):
            try:
                stop_waypoints = traffic_light.get_stop_waypoints()
            except (AttributeError, RuntimeError):
                continue

            for stop_waypoint in stop_waypoints or []:
                for approach in stop_waypoint.previous(
                    self.APPROACH_DISTANCE_METERS
                ):
                    if getattr(approach, "is_junction", False):
                        continue

                    self.original_state = traffic_light.get_state()
                    red_state = self._named_state(
                        self.original_state,
                        "Red"
                    )

                    if red_state is None:
                        continue

                    try:
                        traffic_light.set_state(red_state)
                        traffic_light.freeze(True)
                        ego_vehicle.set_transform(
                            self._elevated_transform(approach)
                        )
                        self._stop_vehicle(ego_vehicle)
                        world.wait_for_tick()
                    except (AttributeError, RuntimeError):
                        self._restore_light(traffic_light)
                        continue

                    self.traffic_light = traffic_light
                    print(
                        "SCENARIO_EVENT: Red traffic light placed 28 m "
                        "ahead and frozen on red",
                        flush=True
                    )
                    return []

        raise RuntimeError(
            "Red Traffic Light could not find a usable signal approach"
        )

    def close(self):
        if self.traffic_light is not None:
            self._restore_light(self.traffic_light)
            self.traffic_light = None

    def _restore_light(self, traffic_light):
        try:
            traffic_light.freeze(False)

            if self.original_state is not None:
                traffic_light.set_state(self.original_state)
        except (AttributeError, RuntimeError):
            pass

    @staticmethod
    def _traffic_lights(world):
        actors = world.get_actors()
        actor_filter = getattr(actors, "filter", None)

        if callable(actor_filter):
            return actors.filter("traffic.traffic_light*")

        return [
            actor
            for actor in actors
            if getattr(actor, "type_id", "").startswith(
                "traffic.traffic_light"
            )
        ]

    @staticmethod
    def _named_state(current_state, state_name):
        return getattr(type(current_state), state_name, None)

    @staticmethod
    def _elevated_transform(waypoint):
        transform = waypoint.transform
        transform.location.z += 0.35
        return transform

    @staticmethod
    def _stop_vehicle(vehicle):
        control = vehicle.get_control()
        control.throttle = 0.0
        control.steer = 0.0
        control.brake = 1.0
        control.hand_brake = False
        vehicle.apply_control(control)
