import math

from config import (
    CROSS_TRAFFIC_ASSUMED_EGO_SPEED_MPS,
    CROSS_TRAFFIC_BRAKE,
    CROSS_TRAFFIC_BRAKE_TIME_GAP_SECONDS,
    CROSS_TRAFFIC_DETECTION_RADIUS_METERS,
    CROSS_TRAFFIC_HOLD_BRAKE,
    CROSS_TRAFFIC_HOLD_SPEED_KMH,
    CROSS_TRAFFIC_MIN_ACTOR_SPEED_MPS,
    CROSS_TRAFFIC_MIN_BRAKING_ACTOR_SPEED_MPS,
    CROSS_TRAFFIC_MIN_CROSSING_ANGLE_DEGREES,
    CROSS_TRAFFIC_PATH_LOOKAHEAD_METERS,
    CROSS_TRAFFIC_TIME_HORIZON_SECONDS,
    CROSS_TRAFFIC_WAITING_BRAKE_THRESHOLD,
    CROSS_TRAFFIC_WAITING_MAX_SPEED_MPS,
    CROSS_TRAFFIC_WARNING_TIME_GAP_SECONDS
)


class CrossTrafficSafety:
    CLEAR = "CLEAR"
    WARNING = "CROSS_TRAFFIC_WARNING"
    BRAKING = "CROSS_TRAFFIC_BRAKING"
    DISABLED = "DISABLED"

    def inspect(
        self,
        world,
        ego_vehicle,
        active,
        target_waypoint=None
    ):
        if not active:
            return self.information(safety_state=self.DISABLED)

        ego_location = ego_vehicle.get_location()
        ego_direction = self._ego_direction(
            ego_vehicle,
            target_waypoint
        )
        ego_velocity = ego_vehicle.get_velocity()
        ego_speed = max(
            self._length(ego_velocity.x, ego_velocity.y),
            CROSS_TRAFFIC_ASSUMED_EGO_SPEED_MPS
        )

        most_urgent = None

        for actor in self._nearby_vehicles(world):
            if actor.id == ego_vehicle.id:
                continue

            if not getattr(actor, "is_alive", True):
                continue

            candidate = self._predict_conflict(
                ego_location,
                ego_direction,
                ego_speed,
                actor
            )

            if candidate is None:
                continue

            if (
                most_urgent is None
                or candidate["arrival_gap_s"]
                < most_urgent["arrival_gap_s"]
            ):
                most_urgent = candidate

        if most_urgent is None:
            return self.information()

        state = (
            self.BRAKING
            if (
                most_urgent["arrival_gap_s"]
                <= CROSS_TRAFFIC_BRAKE_TIME_GAP_SECONDS
                and most_urgent["actor_speed_mps"]
                >= CROSS_TRAFFIC_MIN_BRAKING_ACTOR_SPEED_MPS
            )
            else self.WARNING
        )

        return self.information(
            safety_state=state,
            actor_id=most_urgent["actor_id"],
            actor_type=most_urgent["actor_type"],
            distance_m=most_urgent["distance_m"],
            ego_time_s=most_urgent["ego_time_s"],
            actor_time_s=most_urgent["actor_time_s"],
            arrival_gap_s=most_urgent["arrival_gap_s"],
            event_key=(most_urgent["actor_id"], state)
        )

    def apply(self, requested_control, information, speed_kmh):
        if information["safety_state"] != self.BRAKING:
            return requested_control

        brake = (
            CROSS_TRAFFIC_HOLD_BRAKE
            if speed_kmh <= CROSS_TRAFFIC_HOLD_SPEED_KMH
            else CROSS_TRAFFIC_BRAKE
        )

        return self._copy_control(
            requested_control,
            throttle=0.0,
            brake=max(requested_control.brake, brake)
        )

    def _predict_conflict(
        self,
        ego_location,
        ego_direction,
        ego_speed,
        actor
    ):
        actor_location = actor.get_location()
        offset_x = actor_location.x - ego_location.x
        offset_y = actor_location.y - ego_location.y
        distance = self._length(offset_x, offset_y)

        if distance > CROSS_TRAFFIC_DETECTION_RADIUS_METERS:
            return None

        actor_velocity = actor.get_velocity()
        actor_speed = self._length(
            actor_velocity.x,
            actor_velocity.y
        )

        if actor_speed < CROSS_TRAFFIC_MIN_ACTOR_SPEED_MPS:
            return None

        if self._is_waiting(actor, actor_speed):
            return None

        actor_direction = (
            actor_velocity.x / actor_speed,
            actor_velocity.y / actor_speed
        )
        crossing_sine = abs(
            self._cross(ego_direction, actor_direction)
        )
        minimum_crossing_sine = math.sin(
            math.radians(
                CROSS_TRAFFIC_MIN_CROSSING_ANGLE_DEGREES
            )
        )

        if crossing_sine < minimum_crossing_sine:
            return None

        denominator = self._cross(
            ego_direction,
            actor_direction
        )
        offset = (offset_x, offset_y)
        ego_distance_to_conflict = (
            self._cross(offset, actor_direction)
            / denominator
        )
        actor_distance_to_conflict = (
            self._cross(offset, ego_direction)
            / denominator
        )

        if not (
            0.0 <= ego_distance_to_conflict
            <= CROSS_TRAFFIC_PATH_LOOKAHEAD_METERS
        ):
            return None

        if actor_distance_to_conflict < 0.0:
            return None

        ego_time = ego_distance_to_conflict / ego_speed
        actor_time = actor_distance_to_conflict / actor_speed

        if (
            ego_time > CROSS_TRAFFIC_TIME_HORIZON_SECONDS
            or actor_time > CROSS_TRAFFIC_TIME_HORIZON_SECONDS
        ):
            return None

        arrival_gap = abs(ego_time - actor_time)

        if arrival_gap > CROSS_TRAFFIC_WARNING_TIME_GAP_SECONDS:
            return None

        return {
            "actor_id": getattr(actor, "id", None),
            "actor_type": getattr(actor, "type_id", "vehicle"),
            "distance_m": distance,
            "ego_time_s": ego_time,
            "actor_time_s": actor_time,
            "arrival_gap_s": arrival_gap,
            "actor_speed_mps": actor_speed
        }

    @staticmethod
    def _is_waiting(actor, actor_speed):
        if actor_speed > CROSS_TRAFFIC_WAITING_MAX_SPEED_MPS:
            return False

        try:
            control = actor.get_control()

            if (
                getattr(control, "brake", 0.0)
                >= CROSS_TRAFFIC_WAITING_BRAKE_THRESHOLD
            ):
                return True
        except (AttributeError, RuntimeError):
            pass

        try:
            if actor.is_at_traffic_light():
                state = actor.get_traffic_light_state()
                state_name = getattr(state, "name", str(state))

                if str(state_name).upper() == "RED":
                    return True
        except (AttributeError, RuntimeError):
            pass

        return False

    @staticmethod
    def _nearby_vehicles(world):
        actors = world.get_actors()
        actor_filter = getattr(actors, "filter", None)

        if callable(actor_filter):
            return actors.filter("vehicle.*")

        return [
            actor
            for actor in actors
            if getattr(actor, "type_id", "").startswith("vehicle.")
        ]

    @staticmethod
    def _ego_direction(ego_vehicle, target_waypoint):
        ego_location = ego_vehicle.get_location()

        if target_waypoint is not None:
            target_location = target_waypoint.transform.location
            dx = target_location.x - ego_location.x
            dy = target_location.y - ego_location.y
            length = math.hypot(dx, dy)

            if length > 0.01:
                return dx / length, dy / length

        yaw = math.radians(
            ego_vehicle.get_transform().rotation.yaw
        )
        return math.cos(yaw), math.sin(yaw)

    @classmethod
    def information(
        cls,
        safety_state=CLEAR,
        actor_id=None,
        actor_type=None,
        distance_m=None,
        ego_time_s=None,
        actor_time_s=None,
        arrival_gap_s=None,
        event_key=None
    ):
        return {
            "safety_state": safety_state,
            "actor_id": actor_id,
            "actor_type": actor_type,
            "distance_m": distance_m,
            "ego_time_s": ego_time_s,
            "actor_time_s": actor_time_s,
            "arrival_gap_s": arrival_gap_s,
            "event_key": event_key
        }

    @staticmethod
    def _length(x, y):
        return math.hypot(x, y)

    @staticmethod
    def _cross(first, second):
        return first[0] * second[1] - first[1] * second[0]

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
