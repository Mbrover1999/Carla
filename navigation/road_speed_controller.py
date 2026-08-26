from config import (
    ROAD_SPEED_BRAKE_GAIN,
    ROAD_SPEED_DEADBAND_KMH,
    ROAD_SPEED_DEFAULT_LIMIT_KMH,
    ROAD_SPEED_JUNCTION_TARGET_KMH,
    ROAD_SPEED_LIMIT_FACTOR,
    ROAD_SPEED_MAX_BRAKE,
    ROAD_SPEED_MAX_TARGET_KMH,
    ROAD_SPEED_MAX_THROTTLE,
    ROAD_SPEED_THROTTLE_GAIN
)


class RoadSpeedController:
    def __init__(self):
        self.last_valid_speed_limit = (
            ROAD_SPEED_DEFAULT_LIMIT_KMH
        )

    def apply(
        self,
        vehicle,
        requested_control,
        current_speed_kmh,
        navigation_mode
    ):
        speed_limit = float(vehicle.get_speed_limit() or 0.0)

        if 5.0 <= speed_limit <= 150.0:
            self.last_valid_speed_limit = speed_limit
        else:
            speed_limit = self.last_valid_speed_limit

        target_speed = min(
            speed_limit * ROAD_SPEED_LIMIT_FACTOR,
            ROAD_SPEED_MAX_TARGET_KMH
        )

        if navigation_mode in (
            "APPROACH",
            "INTERSECTION"
        ):
            target_speed = min(
                target_speed,
                ROAD_SPEED_JUNCTION_TARGET_KMH
            )

        speed_error = target_speed - current_speed_kmh

        if speed_error > ROAD_SPEED_DEADBAND_KMH:
            throttle = self._clip(
                speed_error * ROAD_SPEED_THROTTLE_GAIN,
                0.0,
                ROAD_SPEED_MAX_THROTTLE
            )
            brake = 0.0
        elif speed_error < -ROAD_SPEED_DEADBAND_KMH:
            throttle = 0.0
            brake = self._clip(
                -speed_error * ROAD_SPEED_BRAKE_GAIN,
                0.0,
                ROAD_SPEED_MAX_BRAKE
            )
        else:
            throttle = 0.0
            brake = 0.0

        return self._copy_control(
            requested_control,
            throttle=throttle,
            brake=brake
        ), {
            "speed_limit_kmh": speed_limit,
            "target_speed_kmh": target_speed,
            "speed_error_kmh": speed_error
        }

    @staticmethod
    def _clip(value, minimum, maximum):
        return max(minimum, min(value, maximum))

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
