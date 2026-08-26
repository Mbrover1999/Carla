import math

from config import (
    INTERSECTION_AI_BLEND,
    INTERSECTION_MAX_STEERING,
    INTERSECTION_STEERING_GAIN,
    INTERSECTION_STEERING_SMOOTHING
)


class IntersectionController:
    def __init__(self):
        self.previous_steering = 0.0

    def apply(
        self,
        vehicle,
        requested_control,
        target_waypoint
    ):
        if target_waypoint is None:
            return requested_control, self.information()

        vehicle_transform = vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        target_location = target_waypoint.transform.location

        target_yaw = math.degrees(
            math.atan2(
                target_location.y - vehicle_location.y,
                target_location.x - vehicle_location.x
            )
        )

        heading_error = self._normalize_angle(
            target_yaw - vehicle_transform.rotation.yaw
        )

        route_steering = self._clip(
            heading_error * INTERSECTION_STEERING_GAIN,
            -INTERSECTION_MAX_STEERING,
            INTERSECTION_MAX_STEERING
        )

        blended_steering = (
            INTERSECTION_AI_BLEND * requested_control.steer
            + (1.0 - INTERSECTION_AI_BLEND) * route_steering
        )

        smoothed_steering = (
            INTERSECTION_STEERING_SMOOTHING * blended_steering
            + (1.0 - INTERSECTION_STEERING_SMOOTHING)
            * self.previous_steering
        )

        smoothed_steering = self._clip(
            smoothed_steering,
            -INTERSECTION_MAX_STEERING,
            INTERSECTION_MAX_STEERING
        )

        self.previous_steering = smoothed_steering

        return self._copy_control(
            requested_control,
            steer=smoothed_steering
        ), self.information(
            active=True,
            heading_error=heading_error,
            route_steering=route_steering,
            applied_steering=smoothed_steering
        )

    def reset(self, steering=0.0):
        self.previous_steering = steering

    @staticmethod
    def information(
        active=False,
        heading_error=None,
        route_steering=0.0,
        applied_steering=None
    ):
        return {
            "active": active,
            "heading_error_deg": heading_error,
            "route_steering": route_steering,
            "applied_steering": applied_steering
        }

    @staticmethod
    def _normalize_angle(angle_degrees):
        return (
            (angle_degrees + 180.0) % 360.0
        ) - 180.0

    @staticmethod
    def _clip(value, minimum, maximum):
        return max(minimum, min(value, maximum))

    @staticmethod
    def _copy_control(control, steer):
        control_type = type(control)

        return control_type(
            throttle=control.throttle,
            steer=steer,
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
