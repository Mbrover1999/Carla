from config import (
    SAFETY_MIN_SLOW_DISTANCE,
    SAFETY_MIN_BRAKE_DISTANCE,
    SAFETY_MIN_EMERGENCY_DISTANCE,
    SAFETY_SLOW_TIME_GAP,
    SAFETY_BRAKE_TIME_GAP,
    SAFETY_EMERGENCY_TIME_GAP,
    SAFETY_SLOW_THROTTLE,
    SAFETY_BRAKE_AMOUNT,
    SAFETY_EMERGENCY_BRAKE,
    SAFETY_CREEP_MAX_SPEED_KMH,
    SAFETY_CREEP_DISTANCE,
    SAFETY_CREEP_BRAKE_DISTANCE,
    SAFETY_CREEP_THROTTLE
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

        # At walking speed a five-metre gap is not an emergency.  The old
        # fixed threshold kept applying 45% brake forever, so the vehicle
        # could never continue behind slow traffic.  Creep toward a normal
        # standstill gap while preserving the emergency threshold.
        if speed_kmh <= SAFETY_CREEP_MAX_SPEED_KMH:
            if obstacle_distance <= emergency_distance:
                return self._control(
                    requested_control,
                    throttle=0.0,
                    brake=SAFETY_EMERGENCY_BRAKE
                ), "EMERGENCY"

            if obstacle_distance <= SAFETY_CREEP_BRAKE_DISTANCE:
                return self._control(
                    requested_control,
                    throttle=0.0,
                    brake=SAFETY_BRAKE_AMOUNT
                ), "BRAKING"

            if obstacle_distance <= SAFETY_CREEP_DISTANCE:
                return self._control(
                    requested_control,
                    throttle=min(
                        requested_control.throttle,
                        SAFETY_CREEP_THROTTLE
                    ),
                    brake=0.0
                ), "CREEPING"

        if obstacle_distance <= emergency_distance:
            safe_control = self._control(
                requested_control,
                throttle=0.0,
                brake=SAFETY_EMERGENCY_BRAKE
            )

            return safe_control, "EMERGENCY"

        if obstacle_distance <= brake_distance:
            safe_control = self._control(
                requested_control,
                throttle=0.0,
                brake=SAFETY_BRAKE_AMOUNT
            )

            return safe_control, "BRAKING"

        if obstacle_distance <= slow_distance:
            safe_control = self._control(
                requested_control,
                throttle=min(
                    requested_control.throttle,
                    SAFETY_SLOW_THROTTLE
                ),
                brake=0.0
            )

            return safe_control, "SLOWING"

        return requested_control, "CLEAR"

    @staticmethod
    def _control(requested_control, throttle, brake):
        control_type = type(requested_control)

        return control_type(
            throttle=throttle,
            steer=requested_control.steer,
            brake=brake,
            hand_brake=getattr(
                requested_control,
                "hand_brake",
                False
            ),
            reverse=getattr(requested_control, "reverse", False),
            manual_gear_shift=getattr(
                requested_control,
                "manual_gear_shift",
                False
            ),
            gear=getattr(requested_control, "gear", 0)
        )
