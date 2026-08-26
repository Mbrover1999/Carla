import time

from config import (
    CONTROLLER_INACTIVITY_MIN_SPEED_KMH,
    CONTROLLER_INACTIVITY_SAFE_STOP_SECONDS,
    CONTROLLER_INACTIVITY_WARNING_SECONDS
)


class ControllerInactivityDetector:
    NORMAL = "NORMAL"
    WARNING = "INACTIVITY_WARNING"
    SAFE_STOP = "SAFE_STOP"

    def __init__(
        self,
        warning_seconds=(
            CONTROLLER_INACTIVITY_WARNING_SECONDS
        ),
        safe_stop_seconds=(
            CONTROLLER_INACTIVITY_SAFE_STOP_SECONDS
        ),
        minimum_speed_kmh=(
            CONTROLLER_INACTIVITY_MIN_SPEED_KMH
        )
    ):
        if warning_seconds < 0:
            raise ValueError(
                "warning_seconds cannot be negative"
            )

        if safe_stop_seconds <= warning_seconds:
            raise ValueError(
                "safe_stop_seconds must be greater than "
                "warning_seconds"
            )

        self.warning_seconds = warning_seconds
        self.safe_stop_seconds = safe_stop_seconds
        self.minimum_speed_kmh = minimum_speed_kmh

        self.last_response_time = None
        self.state = self.NORMAL

    def update(
        self,
        controller_responded,
        speed_kmh,
        now=None
    ):
        current_time = (
            time.monotonic()
            if now is None
            else float(now)
        )

        if controller_responded:
            self.last_response_time = current_time
            self.state = self.NORMAL
            return self.state, 0.0

        inactive_seconds = self._inactive_seconds(
            current_time
        )

        # Once a safe stop is requested, keep it latched even after
        # the vehicle becomes stationary. Only a fresh controller
        # response may release it.
        if self.state == self.SAFE_STOP:
            return self.state, inactive_seconds

        if speed_kmh < self.minimum_speed_kmh:
            self.reset(now=current_time)
            return self.state, 0.0

        if self.last_response_time is None:
            self.last_response_time = current_time

        inactive_seconds = self._inactive_seconds(
            current_time
        )

        if inactive_seconds >= self.safe_stop_seconds:
            self.state = self.SAFE_STOP
        elif inactive_seconds >= self.warning_seconds:
            self.state = self.WARNING
        else:
            self.state = self.NORMAL

        return self.state, inactive_seconds

    def reset(self, now=None):
        self.last_response_time = (
            time.monotonic()
            if now is None
            else float(now)
        )
        self.state = self.NORMAL

    def _inactive_seconds(self, current_time):
        if self.last_response_time is None:
            return 0.0

        return max(
            0.0,
            current_time - self.last_response_time
        )
