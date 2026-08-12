import carla

from config import (
    SAFETY_MIN_SLOW_DISTANCE,
    SAFETY_MIN_BRAKE_DISTANCE,
    SAFETY_MIN_EMERGENCY_DISTANCE,
    SAFETY_SLOW_TIME_GAP,
    SAFETY_BRAKE_TIME_GAP,
    SAFETY_EMERGENCY_TIME_GAP,
    SAFETY_SLOW_THROTTLE,
    SAFETY_BRAKE_AMOUNT,
    SAFETY_EMERGENCY_BRAKE
)


class SafetyLayer:
    def apply(
        self,
        requested_control,
        obstacle_distance,
        speed_kmh
    ):
        if obstacle_distance is None:
            return requested_control, "CLEAR"

        speed_mps = speed_kmh / 3.6

        slow_distance = max(
            SAFETY_MIN_SLOW_DISTANCE,
            speed_mps * SAFETY_SLOW_TIME_GAP
        )

        brake_distance = max(
            SAFETY_MIN_BRAKE_DISTANCE,
            speed_mps * SAFETY_BRAKE_TIME_GAP
        )

        emergency_distance = max(
            SAFETY_MIN_EMERGENCY_DISTANCE,
            speed_mps * SAFETY_EMERGENCY_TIME_GAP
        )

        if obstacle_distance <= emergency_distance:
            safe_control = carla.VehicleControl(
                steer=requested_control.steer,
                throttle=0.0,
                brake=SAFETY_EMERGENCY_BRAKE
            )

            return safe_control, "EMERGENCY"

        if obstacle_distance <= brake_distance:
            safe_control = carla.VehicleControl(
                steer=requested_control.steer,
                throttle=0.0,
                brake=SAFETY_BRAKE_AMOUNT
            )

            return safe_control, "BRAKING"

        if obstacle_distance <= slow_distance:
            safe_control = carla.VehicleControl(
                steer=requested_control.steer,
                throttle=min(
                    requested_control.throttle,
                    SAFETY_SLOW_THROTTLE
                ),
                brake=0.0
            )

            return safe_control, "SLOWING"

        return requested_control, "CLEAR"