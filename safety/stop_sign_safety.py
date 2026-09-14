import math
import time

from config import (
    STOP_SIGN_APPROACH_RANGE_METERS,
    STOP_SIGN_APPROACH_THROTTLE,
    STOP_SIGN_ASSUMED_DECELERATION_MPS2,
    STOP_SIGN_BRAKE_AMOUNT,
    STOP_SIGN_CREEP_MAX_SPEED_KMH,
    STOP_SIGN_CREEP_TARGET_SPEED_KMH,
    STOP_SIGN_CREEP_THROTTLE,
    STOP_SIGN_DETECTION_RANGE_METERS,
    STOP_SIGN_HOLD_BRAKE,
    STOP_SIGN_HOLD_SECONDS,
    STOP_SIGN_HOLD_SPEED_KMH,
    STOP_SIGN_LANDMARK_TYPE,
    STOP_SIGN_LINE_MARGIN_METERS,
    STOP_SIGN_MIN_BRAKING_RANGE_METERS,
    STOP_SIGN_REACTION_TIME_SECONDS,
    STOP_SIGN_RELEASE_CLEARANCE_METERS,
    STOP_SIGN_STOP_TOLERANCE_METERS
)


class StopSignSafety:
    """Map-based stop-sign detection and complete-stop state machine."""

    CLEAR = "CLEAR"
    APPROACHING = "STOP_SIGN_APPROACHING"
    BRAKING = "STOP_SIGN_BRAKING"
    CREEPING = "STOP_SIGN_CREEPING"
    HOLDING = "STOP_SIGN_HOLDING"
    RELEASED = "STOP_SIGN_RELEASED"
    DISABLED = "DISABLED"

    def __init__(self, clock=None):
        self.clock = clock or time.monotonic
        self.active_sign_id = None
        self.target_location = None
        self.last_route_distance_m = None
        self.hold_started_at = None
        self.released = False
        self.cached_stop_actors = None
        self.cached_global_landmarks = None

    def inspect(
        self,
        world,
        world_map,
        ego_vehicle,
        speed_kmh,
        active=True
    ):
        if not active:
            self.reset()
            return self.information(safety_state=self.DISABLED)

        detection = self._detect_ahead(
            world=world,
            world_map=world_map,
            ego_vehicle=ego_vehicle
        )

        if self.active_sign_id is None:
            if detection is None:
                return self.information()

            self.active_sign_id = detection["sign_id"]
            self.target_location = detection["location"]
            self.last_route_distance_m = detection["distance_m"]
            self.hold_started_at = None
            self.released = False
        elif (
            detection is not None
            and detection["sign_id"] == self.active_sign_id
        ):
            self.target_location = detection["location"]
            self.last_route_distance_m = detection["distance_m"]

        distance_m = self._current_distance(
            ego_vehicle=ego_vehicle,
            detection=detection
        )

        if self.released:
            if (
                distance_m is None
                or distance_m < -STOP_SIGN_RELEASE_CLEARANCE_METERS
            ):
                self.reset()
                return self.information()

            return self.information(
                safety_state=self.RELEASED,
                sign_id=self.active_sign_id,
                distance_m=distance_m,
                event_key=("stop_sign", self.active_sign_id, self.RELEASED)
            )

        now = self.clock()

        if self.hold_started_at is not None:
            hold_elapsed = now - self.hold_started_at
            hold_remaining = max(0.0, STOP_SIGN_HOLD_SECONDS - hold_elapsed)

            if hold_remaining <= 0.0:
                self.released = True
                return self.information(
                    safety_state=self.RELEASED,
                    sign_id=self.active_sign_id,
                    distance_m=distance_m,
                    event_key=(
                        "stop_sign",
                        self.active_sign_id,
                        self.RELEASED
                    )
                )

            return self.information(
                safety_state=self.HOLDING,
                sign_id=self.active_sign_id,
                distance_m=distance_m,
                hold_remaining_s=hold_remaining,
                event_key=("stop_sign", self.active_sign_id, self.HOLDING)
            )

        if (
            distance_m is not None
            and distance_m <= STOP_SIGN_STOP_TOLERANCE_METERS
            and speed_kmh <= STOP_SIGN_HOLD_SPEED_KMH
        ):
            self.hold_started_at = now
            return self.information(
                safety_state=self.HOLDING,
                sign_id=self.active_sign_id,
                distance_m=distance_m,
                hold_remaining_s=STOP_SIGN_HOLD_SECONDS,
                event_key=("stop_sign", self.active_sign_id, self.HOLDING)
            )

        braking_range = self._braking_range(speed_kmh)

        if distance_m is None:
            state = self.CLEAR
        elif (
            speed_kmh <= STOP_SIGN_CREEP_MAX_SPEED_KMH
            and distance_m > STOP_SIGN_STOP_TOLERANCE_METERS
            and distance_m <= braking_range
        ):
            state = self.CREEPING
        elif distance_m <= braking_range:
            state = self.BRAKING
        elif distance_m <= STOP_SIGN_APPROACH_RANGE_METERS:
            state = self.APPROACHING
        else:
            state = self.CLEAR

        return self.information(
            safety_state=state,
            sign_id=self.active_sign_id,
            distance_m=distance_m,
            event_key=("stop_sign", self.active_sign_id, state)
            if state != self.CLEAR
            else None
        )

    def apply(self, requested_control, information, speed_kmh):
        state = information["safety_state"]

        if state in (self.CLEAR, self.RELEASED, self.DISABLED):
            return requested_control

        if state == self.APPROACHING:
            return self._copy_control(
                requested_control,
                throttle=min(
                    requested_control.throttle,
                    STOP_SIGN_APPROACH_THROTTLE
                ),
                brake=requested_control.brake
            )

        if state == self.CREEPING:
            if requested_control.brake > 0.0:
                return requested_control

            return self._copy_control(
                requested_control,
                throttle=(
                    min(
                        requested_control.throttle,
                        STOP_SIGN_CREEP_THROTTLE
                    )
                    if speed_kmh < STOP_SIGN_CREEP_TARGET_SPEED_KMH
                    else 0.0
                ),
                brake=0.0
            )

        brake = (
            STOP_SIGN_HOLD_BRAKE
            if state == self.HOLDING
            else STOP_SIGN_BRAKE_AMOUNT
        )
        return self._copy_control(
            requested_control,
            throttle=0.0,
            brake=max(requested_control.brake, brake)
        )

    def _detect_ahead(self, world, world_map, ego_vehicle):
        waypoint = world_map.get_waypoint(
            ego_vehicle.get_location(),
            project_to_road=True
        )

        if waypoint is None:
            return None

        try:
            landmarks = waypoint.get_landmarks_of_type(
                STOP_SIGN_DETECTION_RANGE_METERS,
                STOP_SIGN_LANDMARK_TYPE,
                False
            )
        except TypeError:
            landmarks = waypoint.get_landmarks_of_type(
                STOP_SIGN_DETECTION_RANGE_METERS,
                STOP_SIGN_LANDMARK_TYPE
            )
        except (AttributeError, RuntimeError):
            landmarks = []

        candidates = []

        for landmark in landmarks or []:
            try:
                route_distance = float(landmark.distance)
                location = landmark.transform.location
            except (AttributeError, TypeError, ValueError):
                continue

            distance = self._front_bumper_distance(
                ego_vehicle,
                route_distance
            )
            candidates.append({
                "sign_id": str(getattr(landmark, "id", "stop")),
                "location": location,
                "distance_m": distance
            })

        actor_candidate = self._detect_actor_fallback(
            world=world,
            world_map=world_map,
            ego_vehicle=ego_vehicle,
            ego_waypoint=waypoint
        )

        if actor_candidate is not None:
            candidates.append(actor_candidate)

        if not candidates:
            global_candidate = self._detect_global_landmark_fallback(
                world_map=world_map,
                ego_vehicle=ego_vehicle,
                ego_waypoint=waypoint
            )

            if global_candidate is not None:
                candidates.append(global_candidate)

        return min(
            candidates,
            default=None,
            key=lambda item: item["distance_m"]
        )

    def _detect_actor_fallback(
        self,
        world,
        world_map,
        ego_vehicle,
        ego_waypoint
    ):
        if self.cached_stop_actors is None:
            try:
                actors = world.get_actors()
                actor_filter = getattr(actors, "filter", None)
                self.cached_stop_actors = list(
                    actor_filter("traffic.stop*")
                    if callable(actor_filter)
                    else [
                        actor for actor in actors
                        if "stop" in getattr(actor, "type_id", "").lower()
                    ]
                )
            except (AttributeError, RuntimeError):
                self.cached_stop_actors = []

        ego_transform = ego_vehicle.get_transform()
        ego_location = ego_transform.location
        yaw = math.radians(ego_transform.rotation.yaw)
        forward = (math.cos(yaw), math.sin(yaw))
        candidates = []

        for actor in self.cached_stop_actors:
            try:
                transform = actor.get_transform()
                trigger = getattr(actor, "trigger_volume", None)
                location = (
                    transform.transform(trigger.location)
                    if trigger is not None
                    else transform.location
                )
                sign_waypoint = world_map.get_waypoint(
                    location,
                    project_to_road=True
                )
            except (AttributeError, RuntimeError):
                continue

            if sign_waypoint is None or not self._same_direction(
                ego_waypoint,
                sign_waypoint
            ):
                continue

            dx = location.x - ego_location.x
            dy = location.y - ego_location.y
            centre_distance = dx * forward[0] + dy * forward[1]
            lateral_distance = abs(
                dx * -forward[1] + dy * forward[0]
            )
            lane_width = float(getattr(ego_waypoint, "lane_width", 3.5))
            distance = self._front_bumper_distance(
                ego_vehicle,
                centre_distance
            )

            if (
                0.0 <= distance <= STOP_SIGN_DETECTION_RANGE_METERS
                and lateral_distance <= max(2.5, lane_width * 0.75)
            ):
                candidates.append({
                    "sign_id": str(getattr(actor, "id", "stop")),
                    "location": location,
                    "distance_m": distance
                })

        return min(
            candidates,
            default=None,
            key=lambda item: item["distance_m"]
        )

    def _detect_global_landmark_fallback(
        self,
        world_map,
        ego_vehicle,
        ego_waypoint
    ):
        if self.cached_global_landmarks is None:
            try:
                self.cached_global_landmarks = list(
                    world_map.get_all_landmarks_of_type(
                        STOP_SIGN_LANDMARK_TYPE
                    )
                )
            except (AttributeError, RuntimeError, TypeError):
                try:
                    self.cached_global_landmarks = [
                        landmark
                        for landmark in world_map.get_all_landmarks()
                        if str(getattr(landmark, "type", ""))
                        == STOP_SIGN_LANDMARK_TYPE
                    ]
                except (AttributeError, RuntimeError, TypeError):
                    self.cached_global_landmarks = []

        transform = ego_vehicle.get_transform()
        ego_location = transform.location
        yaw = math.radians(transform.rotation.yaw)
        forward = (math.cos(yaw), math.sin(yaw))
        right = (-math.sin(yaw), math.cos(yaw))
        ego_lane = getattr(ego_waypoint, "lane_id", 0)
        lane_width = float(getattr(ego_waypoint, "lane_width", 3.5))
        candidates = []

        for landmark in self.cached_global_landmarks:
            try:
                location = landmark.transform.location
            except AttributeError:
                continue

            lane_validities = getattr(landmark, "get_lane_validities", None)

            if callable(lane_validities):
                try:
                    valid_ranges = list(lane_validities())
                except (RuntimeError, TypeError):
                    valid_ranges = []

                if valid_ranges and not any(
                    min(first, last) <= ego_lane <= max(first, last)
                    for first, last in valid_ranges
                ):
                    continue

            dx = location.x - ego_location.x
            dy = location.y - ego_location.y
            centre_distance = dx * forward[0] + dy * forward[1]
            lateral_distance = abs(dx * right[0] + dy * right[1])
            distance = self._front_bumper_distance(
                ego_vehicle,
                centre_distance
            )

            if (
                0.0 <= distance <= STOP_SIGN_DETECTION_RANGE_METERS
                and lateral_distance <= max(5.0, lane_width * 1.5)
            ):
                candidates.append({
                    "sign_id": str(getattr(landmark, "id", "stop")),
                    "location": location,
                    "distance_m": distance
                })

        return min(
            candidates,
            default=None,
            key=lambda item: item["distance_m"]
        )

    @staticmethod
    def _same_direction(first_waypoint, second_waypoint):
        first_lane = getattr(first_waypoint, "lane_id", 0)
        second_lane = getattr(second_waypoint, "lane_id", 0)

        if (
            first_lane != 0
            and second_lane != 0
            and first_lane * second_lane < 0
        ):
            return False

        try:
            first_yaw = math.radians(
                first_waypoint.transform.rotation.yaw
            )
            second_yaw = math.radians(
                second_waypoint.transform.rotation.yaw
            )
        except AttributeError:
            return True

        alignment = (
            math.cos(first_yaw) * math.cos(second_yaw)
            + math.sin(first_yaw) * math.sin(second_yaw)
        )
        return alignment >= 0.5

    def _current_distance(self, ego_vehicle, detection):
        if (
            detection is not None
            and detection["sign_id"] == self.active_sign_id
        ):
            return detection["distance_m"]

        if self.target_location is None:
            return self.last_route_distance_m

        transform = ego_vehicle.get_transform()
        location = transform.location
        yaw = math.radians(transform.rotation.yaw)
        forward = (math.cos(yaw), math.sin(yaw))
        dx = self.target_location.x - location.x
        dy = self.target_location.y - location.y
        centre_distance = dx * forward[0] + dy * forward[1]
        return self._front_bumper_distance(
            ego_vehicle,
            centre_distance
        )

    @staticmethod
    def _front_bumper_distance(ego_vehicle, centre_distance):
        extent = getattr(
            getattr(ego_vehicle, "bounding_box", None),
            "extent",
            None
        )
        front_extent = max(1.0, float(getattr(extent, "x", 2.2)))
        return centre_distance - front_extent - STOP_SIGN_LINE_MARGIN_METERS

    @staticmethod
    def _braking_range(speed_kmh):
        speed_mps = max(0.0, speed_kmh / 3.6)
        stopping_distance = (
            speed_mps * STOP_SIGN_REACTION_TIME_SECONDS
            + speed_mps ** 2
            / (2.0 * STOP_SIGN_ASSUMED_DECELERATION_MPS2)
        )
        return STOP_SIGN_STOP_TOLERANCE_METERS + max(
            STOP_SIGN_MIN_BRAKING_RANGE_METERS,
            stopping_distance
        )

    def reset(self):
        self.active_sign_id = None
        self.target_location = None
        self.last_route_distance_m = None
        self.hold_started_at = None
        self.released = False

    @classmethod
    def information(
        cls,
        safety_state=CLEAR,
        sign_id=None,
        distance_m=None,
        hold_remaining_s=None,
        event_key=None
    ):
        return {
            "safety_state": safety_state,
            "sign_id": sign_id,
            "distance_m": distance_m,
            "hold_remaining_s": hold_remaining_s,
            "event_key": event_key
        }

    @staticmethod
    def _copy_control(control, throttle, brake):
        control_type = type(control)
        return control_type(
            throttle=throttle,
            steer=control.steer,
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
