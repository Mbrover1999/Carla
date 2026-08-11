import carla

from config import (
    SAFETY_SLOW_DISTANCE,
    SAFETY_BRAKE_DISTANCE,
    SAFETY_EMERGENCY_DISTANCE,
    SAFETY_SLOW_THROTTLE,
    SAFETY_BRAKE_AMOUNT,
    SAFETY_EMERGENCY_BRAKE
)


class SafetyLayer:
    def apply(
        self,
        requested_control,
        obstacle_distance
    ):
        if obstacle_distance is None:
            return requested_control, "CLEAR"

        if obstacle_distance <= SAFETY_EMERGENCY_DISTANCE:
            safe_control = carla.VehicleControl(
                steer=requested_control.steer,
                throttle=0.0,
                brake=SAFETY_EMERGENCY_BRAKE
            )

            return safe_control, "EMERGENCY"

        if obstacle_distance <= SAFETY_BRAKE_DISTANCE:
            safe_control = carla.VehicleControl(
                steer=requested_control.steer,
                throttle=0.0,
                brake=SAFETY_BRAKE_AMOUNT
            )

            return safe_control, "BRAKING"

        if obstacle_distance <= SAFETY_SLOW_DISTANCE:
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