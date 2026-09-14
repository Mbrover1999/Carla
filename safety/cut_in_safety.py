import math

from config import (
    CUT_IN_BRAKE_AMOUNT,
    CUT_IN_BRAKE_TIME_SECONDS,
    CUT_IN_CORRIDOR_MARGIN_METERS,
    CUT_IN_EMERGENCY_BRAKE,
    CUT_IN_EMERGENCY_LONGITUDINAL_METERS,
    CUT_IN_EMERGENCY_TIME_SECONDS,
    CUT_IN_FORWARD_RANGE_METERS,
    CUT_IN_MIN_HEADING_ALIGNMENT,
    CUT_IN_MIN_LATERAL_APPROACH_MPS,
    CUT_IN_REAR_RANGE_METERS,
    CUT_IN_TIME_HORIZON_SECONDS,
    CUT_IN_WARNING_THROTTLE
)


class CutInSafety:
    """Predict vehicles entering the ego lane from an adjacent lane."""

    CLEAR = "CLEAR"
    WARNING = "CUT_IN_WARNING"
    BRAKING = "CUT_IN_BRAKING"
    EMERGENCY = "CUT_IN_EMERGENCY"
    DISABLED = "DISABLED"

    def inspect(self, world, ego_vehicle, active=True):
        if not active or world is None:
            return self.information(safety_state=self.DISABLED)

        ego_transform = ego_vehicle.get_transform()
        ego_location = ego_transform.location
        yaw = math.radians(ego_transform.rotation.yaw)
        forward = (math.cos(yaw), math.sin(yaw))
        right = (-math.sin(yaw), math.cos(yaw))
        ego_velocity = ego_vehicle.get_velocity()
        ego_forward_speed = self._dot_velocity(ego_velocity, forward)
        ego_lateral_speed = self._dot_velocity(ego_velocity, right)
        ego_half_width, ego_half_length = self._vehicle_extents(
            ego_vehicle
        )
        most_urgent = None

        for actor in self._nearby_vehicles(world):
            if getattr(actor, "id", None) == getattr(
                ego_vehicle,
                "id",
                None
            ):
                continue

            if not getattr(actor, "is_alive", True):
                continue

            try:
                candidate = self._predict_intrusion(
                    actor=actor,
                    ego_location=ego_location,
                    forward=forward,
                    right=right,
                    ego_forward_speed=ego_forward_speed,
                    ego_lateral_speed=ego_lateral_speed,
                    ego_half_width=ego_half_width,
                    ego_half_length=ego_half_length
                )
            except (AttributeError, RuntimeError, TypeError, ValueError):
                continue

            if candidate is None:
                continue

            if (
                most_urgent is None
                or candidate["urgency"] < most_urgent["urgency"]
            ):
                most_urgent = candidate

        if most_urgent is None:
            return self.information()

        intrusion_time = most_urgent["intrusion_time_s"]
        predicted_longitudinal = abs(
            most_urgent["predicted_longitudinal_m"]
        )

        if (
            intrusion_time <= CUT_IN_EMERGENCY_TIME_SECONDS
            and predicted_longitudinal
            <= CUT_IN_EMERGENCY_LONGITUDINAL_METERS
        ):
            state = self.EMERGENCY
        elif intrusion_time <= CUT_IN_BRAKE_TIME_SECONDS:
            state = self.BRAKING
        else:
            state = self.WARNING

        return self.information(
            safety_state=state,
            actor_id=most_urgent["actor_id"],
            actor_type=most_urgent["actor_type"],
            side=most_urgent["side"],
            distance_m=most_urgent["distance_m"],
            lateral_distance_m=most_urgent["lateral_distance_m"],
            lateral_approach_mps=most_urgent["lateral_approach_mps"],
            intrusion_time_s=intrusion_time,
            predicted_longitudinal_m=(
                most_urgent["predicted_longitudinal_m"]
            ),
            event_key=(
                "cut_in",
                most_urgent["actor_id"],
                state
            )
        )

    def apply(self, requested_control, information):
        state = information["safety_state"]

        if state in (self.CLEAR, self.DISABLED):
            return requested_control

        if state == self.WARNING:
            return self._copy_control(
                requested_control,
                throttle=min(
                    requested_control.throttle,
                    CUT_IN_WARNING_THROTTLE
                ),
                brake=requested_control.brake
            )

        requested_brake = (
            CUT_IN_EMERGENCY_BRAKE
            if state == self.EMERGENCY
            else CUT_IN_BRAKE_AMOUNT
        )
        return self._copy_control(
            requested_control,
            throttle=0.0,
            brake=max(requested_control.brake, requested_brake)
        )

    def _predict_intrusion(
        self,
        actor,
        ego_location,
        forward,
        right,
        ego_forward_speed,
        ego_lateral_speed,
        ego_half_width,
        ego_half_length
    ):
        actor_location = actor.get_location()
        offset_x = actor_location.x - ego_location.x
        offset_y = actor_location.y - ego_location.y
        longitudinal = offset_x * forward[0] + offset_y * forward[1]
        lateral = offset_x * right[0] + offset_y * right[1]

        if not (
            -CUT_IN_REAR_RANGE_METERS
            <= longitudinal
            <= CUT_IN_FORWARD_RANGE_METERS
        ):
            return None

        actor_velocity = actor.get_velocity()
        actor_forward_speed = self._dot_velocity(actor_velocity, forward)
        actor_lateral_speed = self._dot_velocity(actor_velocity, right)
        relative_lateral_speed = actor_lateral_speed - ego_lateral_speed
        side_sign = math.copysign(1.0, lateral)

        # Positive means that the actor is moving toward the centre line of
        # the ego vehicle, regardless of whether it comes from left or right.
        lateral_approach_speed = (
            -side_sign * relative_lateral_speed
            if abs(lateral) > 0.01
            else 0.0
        )
        actor_lateral_approach_speed = (
            -side_sign * actor_lateral_speed
            if abs(lateral) > 0.01
            else 0.0
        )

        # Relative closing alone is insufficient: it could be caused by the
        # ego drifting toward a vehicle that is holding its own lane.  Require
        # the other actor itself to be moving toward the ego lane too.
        if (
            lateral_approach_speed < CUT_IN_MIN_LATERAL_APPROACH_MPS
            or actor_lateral_approach_speed
            < CUT_IN_MIN_LATERAL_APPROACH_MPS
        ):
            return None

        if self._heading_alignment(actor, forward) < (
            CUT_IN_MIN_HEADING_ALIGNMENT
        ):
            return None

        actor_half_width, actor_half_length = self._vehicle_extents(actor)
        lateral_clearance = (
            abs(lateral)
            - ego_half_width
            - actor_half_width
            - CUT_IN_CORRIDOR_MARGIN_METERS
        )
        intrusion_time = max(
            0.0,
            lateral_clearance / lateral_approach_speed
        )

        if intrusion_time > CUT_IN_TIME_HORIZON_SECONDS:
            return None

        relative_forward_speed = actor_forward_speed - ego_forward_speed
        predicted_longitudinal = (
            longitudinal + relative_forward_speed * intrusion_time
        )
        rear_overlap_limit = ego_half_length + actor_half_length + 1.0
        forward_overlap_limit = (
            CUT_IN_FORWARD_RANGE_METERS
            + ego_half_length
            + actor_half_length
        )

        if not (
            -rear_overlap_limit
            <= predicted_longitudinal
            <= forward_overlap_limit
        ):
            return None

        # Prefer the soonest intrusion; for equal times the closer vehicle is
        # the more urgent one.
        urgency = intrusion_time + 0.02 * max(
            0.0,
            abs(predicted_longitudinal) - rear_overlap_limit
        )

        return {
            "actor_id": getattr(actor, "id", None),
            "actor_type": getattr(actor, "type_id", "vehicle"),
            "side": "LEFT" if lateral < 0.0 else "RIGHT",
            "distance_m": math.hypot(offset_x, offset_y),
            "lateral_distance_m": abs(lateral),
            "lateral_approach_mps": lateral_approach_speed,
            "intrusion_time_s": intrusion_time,
            "predicted_longitudinal_m": predicted_longitudinal,
            "urgency": urgency
        }

    @staticmethod
    def _heading_alignment(actor, ego_forward):
        velocity = actor.get_velocity()
        speed = math.hypot(velocity.x, velocity.y)

        if speed > 0.5:
            direction = (velocity.x / speed, velocity.y / speed)
        else:
            yaw = math.radians(actor.get_transform().rotation.yaw)
            direction = (math.cos(yaw), math.sin(yaw))

        return direction[0] * ego_forward[0] + direction[1] * ego_forward[1]

    @staticmethod
    def _dot_velocity(velocity, direction):
        return velocity.x * direction[0] + velocity.y * direction[1]

    @staticmethod
    def _vehicle_extents(vehicle):
        extent = getattr(getattr(vehicle, "bounding_box", None), "extent", None)
        return (
            max(0.5, float(getattr(extent, "y", 0.9))),
            max(1.0, float(getattr(extent, "x", 2.2)))
        )

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

    @classmethod
    def information(
        cls,
        safety_state=CLEAR,
        actor_id=None,
        actor_type=None,
        side=None,
        distance_m=None,
        lateral_distance_m=None,
        lateral_approach_mps=None,
        intrusion_time_s=None,
        predicted_longitudinal_m=None,
        event_key=None
    ):
        return {
            "safety_state": safety_state,
            "actor_id": actor_id,
            "actor_type": actor_type,
            "side": side,
            "distance_m": distance_m,
            "lateral_distance_m": lateral_distance_m,
            "lateral_approach_mps": lateral_approach_mps,
            "intrusion_time_s": intrusion_time_s,
            "predicted_longitudinal_m": predicted_longitudinal_m,
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
