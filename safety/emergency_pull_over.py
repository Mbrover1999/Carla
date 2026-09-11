import math
import time

from config import (
    EMERGENCY_CALL_DELAY_SECONDS,
    EMERGENCY_HORN_INTERVAL_SECONDS,
    EMERGENCY_PULL_OVER_BRAKE,
    EMERGENCY_PULL_OVER_LANE_REACHED_METERS,
    EMERGENCY_PULL_OVER_LANE_CLEARANCE_METERS,
    EMERGENCY_PULL_OVER_FORWARD_CLEARANCE_METERS,
    EMERGENCY_PULL_OVER_REAR_CLEARANCE_METERS,
    EMERGENCY_PULL_OVER_CORRIDOR_HALF_WIDTH_METERS,
    EMERGENCY_PULL_OVER_LOOKAHEAD_METERS,
    EMERGENCY_PULL_OVER_MAX_STEERING,
    EMERGENCY_PULL_OVER_SHOULDER_ENTRY_METERS,
    EMERGENCY_PULL_OVER_STEERING_GAIN,
    EMERGENCY_PULL_OVER_STOPPED_SPEED_KMH,
    EMERGENCY_PULL_OVER_TARGET_SPEED_KMH,
    EMERGENCY_PULL_OVER_THROTTLE
)


class EmergencyPullOverController:
    INACTIVE = "INACTIVE"
    MOVING_RIGHT = "MOVING_TO_RIGHT_LANE"
    MOVING_TO_SHOULDER = "MOVING_TO_SHOULDER"
    WAITING_FOR_RIGHT_LANE = "WAITING_FOR_RIGHT_LANE"
    STOPPING = "STOPPING_ON_RIGHT"
    STOPPED = "STOPPED_WITH_HAZARDS"

    ALLOWED_RIGHT_LANES = {
        "DRIVING",
        "SHOULDER",
        "PARKING"
    }

    def __init__(
        self,
        horn_interval_seconds=EMERGENCY_HORN_INTERVAL_SECONDS,
        call_delay_seconds=EMERGENCY_CALL_DELAY_SECONDS
    ):
        self.horn_interval_seconds = horn_interval_seconds
        self.call_delay_seconds = call_delay_seconds
        self.active = False
        self.phase = self.INACTIVE
        self.target_lane_key = None
        self.last_horn_time = None
        self.call_started = False
        self.environment_obstacles = None
        self.environment_world_id = None

    def apply(
        self,
        vehicle,
        requested_control,
        world_map,
        speed_kmh,
        inactive_seconds,
        now=None,
        world=None
    ):
        current_time = time.monotonic() if now is None else float(now)

        if not self.active:
            self.active = True
            self.last_horn_time = None
            self.call_started = False

        current_waypoint = self._map_waypoint(
            world_map,
            vehicle.get_location()
        )
        target_lane = self._target_lane(current_waypoint)

        if target_lane is None:
            control = self._stopping_control(
                requested_control,
                speed_kmh,
                full_stop=True
            )
            self.phase = (
                self.STOPPED
                if speed_kmh
                <= EMERGENCY_PULL_OVER_STOPPED_SPEED_KMH
                else self.STOPPING
            )
        elif not self._pull_over_path_is_clear(
            world,
            world_map,
            vehicle,
            target_lane
        ):
            control = self._stopping_control(
                requested_control,
                speed_kmh,
                full_stop=True
            )
            self.phase = self.WAITING_FOR_RIGHT_LANE
        else:
            lane_type = self._lane_type_name(target_lane)
            lateral_distance = self._lateral_distance(
                vehicle.get_location(),
                target_lane
            )
            is_shoulder = lane_type in (
                "SHOULDER",
                "PARKING"
            )
            reached_distance = (
                EMERGENCY_PULL_OVER_SHOULDER_ENTRY_METERS
                if is_shoulder
                else EMERGENCY_PULL_OVER_LANE_REACHED_METERS
            )

            if (
                lateral_distance
                <= reached_distance
            ):
                # A shoulder or parking lane is the final destination.
                # Brake as soon as the vehicle enters it; do not continue
                # searching for another lane farther to the right.
                next_right_lane = (
                    None
                    if is_shoulder
                    else self._valid_right_lane(target_lane)
                )

                if next_right_lane is None:
                    self.target_lane_key = None
                    control = self._stopping_control(
                        requested_control,
                        speed_kmh
                    )
                    self.phase = (
                        self.STOPPED
                        if speed_kmh
                        <= EMERGENCY_PULL_OVER_STOPPED_SPEED_KMH
                        else self.STOPPING
                    )
                else:
                    self.target_lane_key = self._lane_key(
                        next_right_lane
                    )
                    target_lane = next_right_lane
                    control = self._lane_change_control(
                        vehicle,
                        requested_control,
                        target_lane,
                        speed_kmh
                    )
                    self.phase = self._moving_phase(target_lane)
            else:
                control = self._lane_change_control(
                    vehicle,
                    requested_control,
                    target_lane,
                    speed_kmh
                )
                self.phase = self._moving_phase(target_lane)

        call_requested = False

        if (
            inactive_seconds >= self.call_delay_seconds
            and not self.call_started
        ):
            self.call_started = True
            call_requested = True

        horn_requested = False

        if call_requested:
            # Give the dialing sound one uninterrupted cycle.
            self.last_horn_time = current_time
        elif self.phase != self.INACTIVE:
            if (
                self.last_horn_time is None
                or current_time - self.last_horn_time
                >= self.horn_interval_seconds
            ):
                horn_requested = True
                self.last_horn_time = current_time

        return control, self.information(
            phase=self.phase,
            hazards_active=True,
            horn_requested=horn_requested,
            call_started=self.call_started,
            call_requested=call_requested,
            call_countdown_seconds=max(
                0.0,
                self.call_delay_seconds - inactive_seconds
            )
        )

    def reset(self):
        self.active = False
        self.phase = self.INACTIVE
        self.target_lane_key = None
        self.last_horn_time = None
        self.call_started = False

    def _target_lane(self, current_waypoint):
        if current_waypoint is None:
            return None

        if self.target_lane_key is not None:
            candidate = current_waypoint

            for _ in range(6):
                if self._lane_key(candidate) == self.target_lane_key:
                    return candidate

                candidate = self._valid_right_lane(candidate)

                if candidate is None:
                    break

        right_lane = self._valid_right_lane(current_waypoint)

        if right_lane is None:
            return None

        self.target_lane_key = self._lane_key(right_lane)
        return right_lane

    def _lane_change_control(
        self,
        vehicle,
        requested_control,
        target_lane,
        speed_kmh
    ):
        steering_waypoint = target_lane

        try:
            future_waypoints = target_lane.next(
                EMERGENCY_PULL_OVER_LOOKAHEAD_METERS
            )

            if future_waypoints:
                steering_waypoint = future_waypoints[0]
        except (AttributeError, RuntimeError):
            pass

        vehicle_transform = vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        target_location = steering_waypoint.transform.location
        target_yaw = math.degrees(
            math.atan2(
                target_location.y - vehicle_location.y,
                target_location.x - vehicle_location.x
            )
        )
        heading_error = self._normalize_angle(
            target_yaw - vehicle_transform.rotation.yaw
        )
        steer = self._clip(
            heading_error * EMERGENCY_PULL_OVER_STEERING_GAIN,
            -EMERGENCY_PULL_OVER_MAX_STEERING,
            EMERGENCY_PULL_OVER_MAX_STEERING
        )

        if requested_control.brake > 0.0:
            throttle = 0.0
            brake = requested_control.brake
        elif speed_kmh > EMERGENCY_PULL_OVER_TARGET_SPEED_KMH + 1.0:
            throttle = 0.0
            brake = EMERGENCY_PULL_OVER_BRAKE
        elif speed_kmh < EMERGENCY_PULL_OVER_TARGET_SPEED_KMH - 1.0:
            throttle = EMERGENCY_PULL_OVER_THROTTLE
            brake = 0.0
        else:
            throttle = min(requested_control.throttle, 0.05)
            brake = 0.0

        return self._copy_control(
            requested_control,
            steer=steer,
            throttle=throttle,
            brake=brake
        )

    @staticmethod
    def _stopping_control(
        requested_control,
        speed_kmh,
        full_stop=False
    ):
        brake = (
            1.0
            if (
                full_stop
                or speed_kmh <= EMERGENCY_PULL_OVER_STOPPED_SPEED_KMH
            )
            else max(requested_control.brake, EMERGENCY_PULL_OVER_BRAKE)
        )

        return EmergencyPullOverController._copy_control(
            requested_control,
            steer=0.0,
            throttle=0.0,
            brake=brake
        )

    @classmethod
    def _valid_right_lane(cls, waypoint):
        try:
            candidate = waypoint.get_right_lane()
        except (AttributeError, RuntimeError):
            return None

        if candidate is None:
            return None

        lane_type = cls._lane_type_name(candidate)

        if lane_type not in cls.ALLOWED_RIGHT_LANES:
            return None

        current_lane_id = getattr(waypoint, "lane_id", 0)
        candidate_lane_id = getattr(candidate, "lane_id", 0)

        if (
            lane_type == "DRIVING"
            and current_lane_id * candidate_lane_id < 0
        ):
            return None

        return candidate

    @classmethod
    def _moving_phase(cls, target_lane):
        return (
            cls.MOVING_TO_SHOULDER
            if cls._lane_type_name(target_lane) in (
                "SHOULDER",
                "PARKING"
            )
            else cls.MOVING_RIGHT
        )

    @staticmethod
    def _lane_type_name(waypoint):
        lane_type = getattr(waypoint, "lane_type", "UNKNOWN")
        name = getattr(lane_type, "name", None)

        if name is None:
            name = str(lane_type).split(".")[-1]

        return name.upper()

    def _pull_over_path_is_clear(
        self,
        world,
        world_map,
        ego_vehicle,
        first_target_lane
    ):
        target_lane = first_target_lane

        for _ in range(4):
            if not self._target_lane_is_clear(
                world,
                world_map,
                ego_vehicle,
                target_lane
            ):
                return False

            if self._lane_type_name(target_lane) in (
                "SHOULDER",
                "PARKING"
            ):
                break

            target_lane = self._valid_right_lane(target_lane)

            if target_lane is None:
                break

        return True

    def _target_lane_is_clear(
        self,
        world,
        world_map,
        ego_vehicle,
        target_lane
    ):
        if world is None:
            return True

        ego_location = ego_vehicle.get_location()
        target_road_id = getattr(target_lane, "road_id", None)
        target_lane_id = getattr(target_lane, "lane_id", None)
        target_transform = target_lane.transform
        target_location = target_transform.location
        target_yaw = math.radians(target_transform.rotation.yaw)
        forward_x = math.cos(target_yaw)
        forward_y = math.sin(target_yaw)
        right_x = -forward_y
        right_y = forward_x

        corridor_points = self._target_corridor_points(
            ego_location,
            target_lane
        )

        for actor in self._potential_obstacles(world):
            if getattr(actor, "id", None) == getattr(
                ego_vehicle,
                "id",
                None
            ):
                continue

            actor_type = getattr(actor, "type_id", "") or ""

            if actor_type and not actor_type.startswith((
                "vehicle.",
                "walker.",
                "static.prop."
            )):
                continue

            try:
                actor_location = self._obstacle_location(actor)

                if actor_location is None:
                    continue

                actor_waypoint = (
                    self._map_waypoint(
                        world_map,
                        actor_location
                    )
                )
            except (AttributeError, RuntimeError):
                continue

            same_target_lane = (
                actor_waypoint is not None
                and getattr(actor_waypoint, "road_id", None)
                == target_road_id
                and getattr(actor_waypoint, "lane_id", None)
                == target_lane_id
            )

            offset_x = actor_location.x - target_location.x
            offset_y = actor_location.y - target_location.y
            longitudinal = offset_x * forward_x + offset_y * forward_y
            lateral = abs(offset_x * right_x + offset_y * right_y)
            inside_target_corridor = (
                lateral
                <= EMERGENCY_PULL_OVER_CORRIDOR_HALF_WIDTH_METERS
                and -EMERGENCY_PULL_OVER_REAR_CLEARANCE_METERS
                <= longitudinal
                <= EMERGENCY_PULL_OVER_FORWARD_CLEARANCE_METERS
            )
            distance = math.hypot(
                actor_location.x - ego_location.x,
                actor_location.y - ego_location.y
            )
            inside_merge_path = (
                self._distance_to_path(
                    actor_location,
                    corridor_points
                )
                <= (
                    EMERGENCY_PULL_OVER_CORRIDOR_HALF_WIDTH_METERS
                    + self._obstacle_radius(actor)
                )
                and distance
                <= EMERGENCY_PULL_OVER_FORWARD_CLEARANCE_METERS + 8.0
            )

            if (
                inside_target_corridor
                or inside_merge_path
                or (
                    same_target_lane
                    and distance
                    <= EMERGENCY_PULL_OVER_LANE_CLEARANCE_METERS
                )
            ):
                return False

        return True

    def _potential_obstacles(self, world):
        actors = list(world.get_actors())
        world_id = id(world)

        if self.environment_world_id != world_id:
            self.environment_world_id = world_id
            self.environment_obstacles = []

            try:
                import carla

                self.environment_obstacles = list(
                    world.get_environment_objects(
                        carla.CityObjectLabel.Vehicles
                    )
                )
            except (ImportError, AttributeError, RuntimeError):
                pass

        return actors + list(self.environment_obstacles or [])

    @classmethod
    def _target_corridor_points(cls, ego_location, target_lane):
        points = [ego_location, target_lane.transform.location]
        current = target_lane
        travelled = 0.0

        while travelled < EMERGENCY_PULL_OVER_FORWARD_CLEARANCE_METERS:
            try:
                candidates = list(current.next(5.0))
            except (AttributeError, RuntimeError):
                break

            if not candidates:
                break

            current_yaw = current.transform.rotation.yaw
            current = min(
                candidates,
                key=lambda candidate: abs(
                    (
                        candidate.transform.rotation.yaw
                        - current_yaw
                        + 180.0
                    ) % 360.0 - 180.0
                )
            )
            points.append(current.transform.location)
            travelled += 5.0

        return points

    @staticmethod
    def _obstacle_location(obstacle):
        get_location = getattr(obstacle, "get_location", None)

        if callable(get_location):
            return get_location()

        transform = getattr(obstacle, "transform", None)

        if transform is not None:
            return getattr(transform, "location", None)

        bounding_box = getattr(obstacle, "bounding_box", None)
        return getattr(bounding_box, "location", None)

    @staticmethod
    def _obstacle_radius(obstacle):
        bounding_box = getattr(obstacle, "bounding_box", None)
        extent = getattr(bounding_box, "extent", None)

        if extent is None:
            return 0.8

        return min(2.5, max(float(extent.x), float(extent.y)))

    @classmethod
    def _distance_to_path(cls, point, path):
        if len(path) < 2:
            return float("inf")

        return min(
            cls._distance_to_segment(point, start, end)
            for start, end in zip(path, path[1:])
        )

    @staticmethod
    def _map_waypoint(world_map, location):
        try:
            import carla

            return world_map.get_waypoint(
                location,
                project_to_road=True,
                lane_type=carla.LaneType.Any
            )
        except (ImportError, AttributeError, RuntimeError, TypeError):
            return world_map.get_waypoint(
                location,
                project_to_road=True
            )

    @staticmethod
    def _distance_to_segment(point, start, end):
        segment_x = end.x - start.x
        segment_y = end.y - start.y
        segment_length_squared = segment_x ** 2 + segment_y ** 2

        if segment_length_squared <= 0.0001:
            return math.hypot(point.x - start.x, point.y - start.y)

        projection = (
            (point.x - start.x) * segment_x
            + (point.y - start.y) * segment_y
        ) / segment_length_squared
        projection = max(0.0, min(projection, 1.0))
        closest_x = start.x + projection * segment_x
        closest_y = start.y + projection * segment_y
        return math.hypot(point.x - closest_x, point.y - closest_y)

    @classmethod
    def _lane_key(cls, waypoint):
        return (
            getattr(waypoint, "lane_id", None),
            cls._lane_type_name(waypoint)
        )

    @staticmethod
    def _lateral_distance(vehicle_location, waypoint):
        waypoint_transform = waypoint.transform
        waypoint_location = waypoint_transform.location
        yaw = math.radians(waypoint_transform.rotation.yaw)
        dx = vehicle_location.x - waypoint_location.x
        dy = vehicle_location.y - waypoint_location.y
        return abs(-math.sin(yaw) * dx + math.cos(yaw) * dy)

    @staticmethod
    def _normalize_angle(angle_degrees):
        return ((angle_degrees + 180.0) % 360.0) - 180.0

    @staticmethod
    def _clip(value, minimum, maximum):
        return max(minimum, min(value, maximum))

    @staticmethod
    def _copy_control(control, steer, throttle, brake):
        control_type = type(control)

        return control_type(
            throttle=throttle,
            steer=steer,
            brake=brake,
            hand_brake=getattr(control, "hand_brake", False),
            reverse=getattr(control, "reverse", False),
            manual_gear_shift=getattr(
                control,
                "manual_gear_shift",
                False
            ),
            gear=getattr(control, "gear", 0)
        )

    @classmethod
    def information(
        cls,
        phase=INACTIVE,
        hazards_active=False,
        horn_requested=False,
        call_started=False,
        call_requested=False,
        call_countdown_seconds=None
    ):
        return {
            "phase": phase,
            "hazards_active": hazards_active,
            "horn_requested": horn_requested,
            "call_started": call_started,
            "call_requested": call_requested,
            "call_countdown_seconds": call_countdown_seconds
        }
