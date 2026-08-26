import math

from config import (
    LANE_KEEPING_HEADING_GAIN,
    LANE_KEEPING_HEADING_THRESHOLD_DEGREES,
    LANE_KEEPING_LATERAL_GAIN,
    LANE_KEEPING_MAX_CORRECTION,
    LANE_KEEPING_MIN_SPEED_KMH,
    LANE_KEEPING_OFFSET_THRESHOLD_METERS,
    LANE_KEEPING_STEERING_LIMIT
)


class LaneKeepingAssist:
    CENTERED = "CENTERED"
    CORRECTING_LEFT = "CORRECTING_LEFT"
    CORRECTING_RIGHT = "CORRECTING_RIGHT"
    LOW_SPEED = "INACTIVE_LOW_SPEED"
    JUNCTION = "INACTIVE_JUNCTION"
    NO_LANE = "NO_LANE"
    DISABLED = "DISABLED"
    SUPPRESSED = "SUPPRESSED"

    def __init__(
        self,
        world_map,
        minimum_speed_kmh=LANE_KEEPING_MIN_SPEED_KMH,
        offset_threshold=(
            LANE_KEEPING_OFFSET_THRESHOLD_METERS
        ),
        heading_threshold=(
            LANE_KEEPING_HEADING_THRESHOLD_DEGREES
        ),
        lateral_gain=LANE_KEEPING_LATERAL_GAIN,
        heading_gain=LANE_KEEPING_HEADING_GAIN,
        maximum_correction=LANE_KEEPING_MAX_CORRECTION,
        steering_limit=LANE_KEEPING_STEERING_LIMIT
    ):
        self.world_map = world_map
        self.minimum_speed_kmh = minimum_speed_kmh
        self.offset_threshold = offset_threshold
        self.heading_threshold = heading_threshold
        self.lateral_gain = lateral_gain
        self.heading_gain = heading_gain
        self.maximum_correction = maximum_correction
        self.steering_limit = steering_limit

    def apply(self, vehicle, requested_control, speed_kmh):
        if speed_kmh < self.minimum_speed_kmh:
            return requested_control, self.status(self.LOW_SPEED)

        vehicle_transform = vehicle.get_transform()

        waypoint = self.world_map.get_waypoint(
            vehicle_transform.location,
            project_to_road=True
        )

        if waypoint is None:
            return requested_control, self.status(self.NO_LANE)

        if waypoint.is_junction:
            return requested_control, self.status(self.JUNCTION)

        lateral_offset = self._calculate_lateral_offset(
            vehicle_transform,
            waypoint.transform
        )

        heading_error = self._normalize_angle(
            vehicle_transform.rotation.yaw
            - waypoint.transform.rotation.yaw
        )

        should_correct = (
            abs(lateral_offset) >= self.offset_threshold
            or abs(heading_error) >= self.heading_threshold
        )

        if not should_correct:
            return requested_control, self.status(
                self.CENTERED,
                lateral_offset=lateral_offset,
                heading_error=heading_error
            )

        raw_correction = -(
            lateral_offset * self.lateral_gain
            + heading_error * self.heading_gain
        )

        correction = self._clip(
            raw_correction,
            -self.maximum_correction,
            self.maximum_correction
        )

        corrected_steering = self._clip(
            requested_control.steer + correction,
            -self.steering_limit,
            self.steering_limit
        )

        corrected_control = self._copy_control(
            requested_control,
            steer=corrected_steering
        )

        state = (
            self.CORRECTING_RIGHT
            if correction > 0
            else self.CORRECTING_LEFT
        )

        return corrected_control, self.status(
            state,
            lateral_offset=lateral_offset,
            heading_error=heading_error,
            correction=correction
        )

    @staticmethod
    def status(
        state,
        lateral_offset=None,
        heading_error=None,
        correction=0.0
    ):
        return {
            "state": state,
            "lateral_offset_m": lateral_offset,
            "heading_error_deg": heading_error,
            "steering_correction": correction
        }

    @staticmethod
    def _calculate_lateral_offset(
        vehicle_transform,
        waypoint_transform
    ):
        delta_x = (
            vehicle_transform.location.x
            - waypoint_transform.location.x
        )
        delta_y = (
            vehicle_transform.location.y
            - waypoint_transform.location.y
        )

        yaw_radians = math.radians(
            waypoint_transform.rotation.yaw
        )

        right_x = -math.sin(yaw_radians)
        right_y = math.cos(yaw_radians)

        return delta_x * right_x + delta_y * right_y

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
