import math

from scenarios.obstacle_ahead import ObstacleAheadScenario
from scenarios.red_traffic_light import RedTrafficLightScenario


class VehicleCutInScenario(ObstacleAheadScenario):
    """A controlled vehicle merges from the left into the ego lane."""

    DISTANCE_CANDIDATES_METERS = (65.0, 75.0, 85.0)
    START_AHEAD_METERS = 14.0
    ROUTE_STEP_METERS = 4.0
    APPROACH_SPEED_MPS = 3.0
    TARGET_SPEED_MPS = 5.5
    THROTTLE = 0.52
    MAX_STEERING = 0.32
    CUT_IN_MIN_GAP_METERS = 9.0
    CUT_IN_MAX_GAP_METERS = 11.0
    CUT_IN_MIN_EGO_SPEED_MPS = 3.0

    def __init__(self):
        self.vehicle = None
        self.ego_vehicle = None
        self.world_map = None
        self.route = []
        self.route_index = 0
        self.merge_announced = False
        self.completed_announced = False
        self.controlled_traffic_lights = []

    def setup(self, world, ego_vehicle):
        world_map = world.get_map()
        self.world_map = world_map
        self.ego_vehicle = ego_vehicle
        ego_waypoint = self._prepare_demo_location(
            world,
            world_map,
            ego_vehicle
        )

        if ego_waypoint is None:
            raise RuntimeError(
                "Vehicle Cut-In could not find a straight two-lane road"
            )

        ego_start = self._straight_waypoint_ahead(
            ego_waypoint,
            self.START_AHEAD_METERS
        )
        left_start = self._left_driving_lane(ego_start)

        if left_start is None:
            raise RuntimeError("Vehicle Cut-In lost the left driving lane")

        blueprint = self._select_blueprint(
            world.get_blueprint_library()
        )
        vehicle = world.try_spawn_actor(
            blueprint,
            self._spawn_transform(left_start)
        )

        if vehicle is None:
            raise RuntimeError("Vehicle Cut-In could not spawn its vehicle")

        vehicle.set_autopilot(False)
        self.vehicle = vehicle
        self._force_nearby_lights_green(world, ego_vehicle.get_location())
        # The merge route is deliberately created later, from the cut-in
        # vehicle's current position, once the ego vehicle has caught up.
        self.route = []
        self.route_index = 0
        self._set_initial_velocity(left_start, self.APPROACH_SPEED_MPS)
        print(
            "SCENARIO_EVENT: Cut-in vehicle is approaching in the left lane",
            flush=True
        )
        return [vehicle]

    def update(self, _elapsed_seconds):
        if self.vehicle is None:
            return

        if not self.merge_announced:
            if not self._ego_is_in_cut_in_position():
                self._follow_left_lane()
                return

            current_waypoint = self._vehicle_waypoint()
            self.route = self._build_merge_route(current_waypoint)

            if len(self.route) < 3:
                self._follow_left_lane()
                return

            self.route_index = 1
            self.merge_announced = True
            print(
                "SCENARIO_EVENT: Vehicle from the left is cutting into the ego lane",
                flush=True
            )

        location = self.vehicle.get_location()

        while self.route_index < len(self.route) - 1:
            target = self.route[self.route_index].transform.location

            if self._location_distance(location, target) > 2.3:
                break

            self.route_index += 1

        target_waypoint = self.route[self.route_index]
        self._drive_toward(target_waypoint)

        if (
            self.route_index == len(self.route) - 1
            and not self.completed_announced
        ):
            self.completed_announced = True
            print(
                "SCENARIO_EVENT: Cut-in vehicle completed the merge; safety distance is closing",
                flush=True
            )

    def _ego_is_in_cut_in_position(self):
        if self.ego_vehicle is None:
            return False

        ego_transform = self.ego_vehicle.get_transform()
        ego_location = ego_transform.location
        actor_location = self.vehicle.get_location()
        yaw = math.radians(ego_transform.rotation.yaw)
        longitudinal_gap = (
            (actor_location.x - ego_location.x) * math.cos(yaw)
            + (actor_location.y - ego_location.y) * math.sin(yaw)
        )
        ego_velocity = self.ego_vehicle.get_velocity()
        ego_speed = math.hypot(ego_velocity.x, ego_velocity.y)
        return (
            self.CUT_IN_MIN_GAP_METERS
            <= longitudinal_gap
            <= self.CUT_IN_MAX_GAP_METERS
            and ego_speed >= self.CUT_IN_MIN_EGO_SPEED_MPS
        )

    def _vehicle_waypoint(self):
        if self.world_map is None:
            return None

        try:
            return self.world_map.get_waypoint(
                self.vehicle.get_location(),
                project_to_road=True
            )
        except (AttributeError, RuntimeError):
            return None

    def _follow_left_lane(self):
        current_waypoint = self._vehicle_waypoint()

        if current_waypoint is None:
            return

        target = self._next_straight(current_waypoint, 8.0)

        if target is not None:
            self._drive_toward(
                target,
                target_speed_mps=self.APPROACH_SPEED_MPS
            )

    def _build_merge_route(self, left_start):
        route = [left_start]
        # Stay in the left lane only briefly after activation, then begin
        # crossing while there is still enough longitudinal safety margin.
        left_ahead = self._next_straight(left_start, 1.0)

        if left_ahead is None:
            return route

        route.append(left_ahead)
        ego_lane = self._right_driving_lane(left_ahead)

        if ego_lane is None:
            return route

        # Aim farther down the ego lane: this makes one smooth diagonal merge
        # instead of a sharp 90-degree turn between lane centres.
        merge_target = self._next_straight(ego_lane, 6.0) or ego_lane
        route.append(merge_target)
        current = merge_target

        for _ in range(8):
            current = self._next_straight(current, self.ROUTE_STEP_METERS)
            if current is None:
                break
            route.append(current)

        return route

    def _drive_toward(self, target_waypoint, target_speed_mps=None):
        transform = self.vehicle.get_transform()
        location = transform.location
        target = target_waypoint.transform.location
        desired_yaw = math.degrees(math.atan2(
            target.y - location.y,
            target.x - location.x
        ))
        heading_error = (
            (desired_yaw - transform.rotation.yaw + 180.0) % 360.0
        ) - 180.0
        steer = self._clip(
            heading_error * 0.022,
            -self.MAX_STEERING,
            self.MAX_STEERING
        )
        velocity = self.vehicle.get_velocity()
        speed = math.hypot(velocity.x, velocity.y)
        target_speed = (
            self.TARGET_SPEED_MPS
            if target_speed_mps is None
            else target_speed_mps
        )

        if speed < target_speed - 0.35:
            throttle, brake = self.THROTTLE, 0.0
        elif speed > target_speed + 0.6:
            throttle, brake = 0.0, 0.22
        else:
            throttle, brake = 0.16, 0.0

        control = self.vehicle.get_control()
        control.steer = steer
        control.throttle = throttle
        control.brake = brake
        control.hand_brake = False
        self.vehicle.apply_control(control)

    def _set_initial_velocity(self, waypoint, speed_mps):
        try:
            yaw = math.radians(waypoint.transform.rotation.yaw)
            velocity = self.vehicle.get_velocity()
            velocity.x = math.cos(yaw) * speed_mps
            velocity.y = math.sin(yaw) * speed_mps
            velocity.z = 0.0
            self.vehicle.set_target_velocity(velocity)
        except (AttributeError, RuntimeError):
            pass

    def _force_nearby_lights_green(self, world, ego_location):
        try:
            actors = world.get_actors().filter("traffic.traffic_light*")
        except (AttributeError, RuntimeError):
            return

        for light in actors:
            try:
                light_location = light.get_location()

                if self._location_distance(ego_location, light_location) > 90.0:
                    continue

                original_state = light.get_state()
                green_state = RedTrafficLightScenario._named_state(
                    original_state,
                    "Green"
                )

                if green_state is None:
                    continue

                self.controlled_traffic_lights.append((
                    light,
                    original_state
                ))
                light.set_state(green_state)
                light.freeze(True)
            except (AttributeError, RuntimeError):
                continue

        if self.controlled_traffic_lights:
            print(
                "SCENARIO_EVENT: Nearby traffic lights held green for the cut-in demo",
                flush=True
            )

    def close(self):
        for light, original_state in self.controlled_traffic_lights:
            try:
                light.freeze(False)
                light.set_state(original_state)
            except (AttributeError, RuntimeError):
                pass

        self.controlled_traffic_lights = []

    @classmethod
    def _is_demo_waypoint_suitable(cls, waypoint):
        return cls._left_driving_lane(waypoint) is not None

    @classmethod
    def _left_driving_lane(cls, waypoint):
        try:
            candidate = waypoint.get_left_lane()
        except (AttributeError, RuntimeError):
            return None
        return cls._same_direction_driving_lane(waypoint, candidate)

    @classmethod
    def _right_driving_lane(cls, waypoint):
        try:
            candidate = waypoint.get_right_lane()
        except (AttributeError, RuntimeError):
            return None
        return cls._same_direction_driving_lane(waypoint, candidate)

    @staticmethod
    def _same_direction_driving_lane(source, candidate):
        if candidate is None:
            return None

        lane_type = getattr(candidate, "lane_type", "Driving")
        lane_type_name = getattr(lane_type, "name", str(lane_type)).upper()
        source_id = getattr(source, "lane_id", 1)
        candidate_id = getattr(candidate, "lane_id", source_id)

        if "DRIVING" not in lane_type_name or source_id * candidate_id < 0:
            return None

        return candidate

    @staticmethod
    def _next_straight(waypoint, distance):
        try:
            candidates = list(waypoint.next(distance))
        except (AttributeError, RuntimeError):
            return None

        if not candidates:
            return None

        return min(
            candidates,
            key=lambda candidate: ObstacleAheadScenario._heading_difference(
                waypoint,
                candidate
            )
        )

    @staticmethod
    def _location_distance(first, second):
        return math.hypot(first.x - second.x, first.y - second.y)

    @staticmethod
    def _clip(value, minimum, maximum):
        return max(minimum, min(maximum, value))
