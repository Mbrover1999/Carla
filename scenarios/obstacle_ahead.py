class ObstacleAheadScenario:
    DISTANCE_CANDIDATES_METERS = (12.0, 14.0, 16.0)
    PREFERRED_BLUEPRINTS = (
        "vehicle.lincoln.mkz_2020",
        "vehicle.audi.tt",
        "vehicle.tesla.model3"
    )

    def setup(self, world, ego_vehicle):
        world_map = world.get_map()
        prepared_waypoint = self._prepare_demo_location(
            world,
            world_map,
            ego_vehicle
        )
        ego_waypoint = world_map.get_waypoint(
            ego_vehicle.get_location(),
            project_to_road=True
        )

        if prepared_waypoint is not None:
            ego_waypoint = prepared_waypoint

        if ego_waypoint is None:
            raise RuntimeError(
                "Obstacle Ahead could not locate the ego vehicle lane"
            )

        blueprint = self._select_blueprint(
            world.get_blueprint_library()
        )

        for distance in self.DISTANCE_CANDIDATES_METERS:
            for waypoint in ego_waypoint.next(distance):
                obstacle = world.try_spawn_actor(
                    blueprint,
                    self._spawn_transform(waypoint)
                )

                if obstacle is None:
                    continue

                self._hold_stationary(obstacle)
                print(
                    "SCENARIO_EVENT: Obstacle vehicle placed "
                    f"{distance:.0f} m ahead",
                    flush=True
                )
                return [obstacle]

        raise RuntimeError(
            "Obstacle Ahead could not place a vehicle in front of the ego "
            "vehicle"
        )

    def _select_blueprint(self, blueprint_library):
        for blueprint_id in self.PREFERRED_BLUEPRINTS:
            try:
                blueprint = blueprint_library.find(blueprint_id)
            except (IndexError, RuntimeError):
                continue

            if blueprint is not None:
                self._configure_blueprint(blueprint)
                return blueprint

        vehicle_blueprints = list(
            blueprint_library.filter("vehicle.*")
        )

        for blueprint in vehicle_blueprints:
            if self._is_four_wheeled(blueprint):
                self._configure_blueprint(blueprint)
                return blueprint

        raise RuntimeError(
            "Obstacle Ahead could not find a vehicle blueprint"
        )

    @staticmethod
    def _is_four_wheeled(blueprint):
        try:
            return int(
                blueprint.get_attribute("number_of_wheels")
            ) == 4
        except (AttributeError, TypeError, ValueError):
            return True

    @staticmethod
    def _configure_blueprint(blueprint):
        try:
            if blueprint.has_attribute("role_name"):
                blueprint.set_attribute(
                    "role_name",
                    "scenario_obstacle"
                )
        except (AttributeError, RuntimeError):
            pass

    @staticmethod
    def _spawn_transform(waypoint):
        transform = waypoint.transform
        transform.location.z += 0.35
        return transform

    @staticmethod
    def _hold_stationary(vehicle):
        vehicle.set_autopilot(False)
        control = vehicle.get_control()
        control.throttle = 0.0
        control.steer = 0.0
        control.brake = 1.0
        control.hand_brake = True
        vehicle.apply_control(control)

        try:
            vehicle.set_simulate_physics(False)
        except (AttributeError, RuntimeError):
            pass

    def _prepare_demo_location(self, world, world_map, ego_vehicle):
        try:
            spawn_points = world_map.get_spawn_points()
        except (AttributeError, RuntimeError):
            return None

        for spawn_point in spawn_points:
            waypoint = world_map.get_waypoint(
                spawn_point.location,
                project_to_road=True
            )

            if waypoint is None or getattr(
                waypoint,
                "is_junction",
                False
            ):
                continue

            candidates = waypoint.next(
                self.DISTANCE_CANDIDATES_METERS[0]
            )
            straight_candidates = [
                candidate
                for candidate in candidates
                if (
                    not getattr(candidate, "is_junction", False)
                    and self._heading_difference(
                        waypoint,
                        candidate
                    ) <= 8.0
                )
            ]

            if not straight_candidates:
                continue

            ego_vehicle.set_transform(spawn_point)
            self._stop_ego_vehicle(ego_vehicle)

            try:
                world.wait_for_tick()
            except (AttributeError, RuntimeError):
                pass

            print(
                "SCENARIO_EVENT: Ego vehicle moved to a clear straight "
                "road",
                flush=True
            )
            return waypoint

        return None

    @staticmethod
    def _heading_difference(first_waypoint, second_waypoint):
        first_yaw = first_waypoint.transform.rotation.yaw
        second_yaw = second_waypoint.transform.rotation.yaw
        difference = (second_yaw - first_yaw + 180.0) % 360.0 - 180.0
        return abs(difference)

    @staticmethod
    def _stop_ego_vehicle(vehicle):
        control = vehicle.get_control()
        control.throttle = 0.0
        control.steer = 0.0
        control.brake = 1.0
        control.hand_brake = False
        vehicle.apply_control(control)
