import math

from config import (
    TRAFFIC_LIGHT_HOLD_BRAKE,
    TRAFFIC_LIGHT_HOLD_SPEED_KMH,
    TRAFFIC_LIGHT_RED_BRAKE
)


class TrafficLightSafety:
    NONE = "NONE"
    RED = "RED"
    YELLOW = "YELLOW"
    GREEN = "GREEN"
    OFF = "OFF"
    UNKNOWN = "UNKNOWN"

    CLEAR = "CLEAR"
    YELLOW_WARNING = "TRAFFIC_LIGHT_YELLOW"
    RED_BRAKING = "TRAFFIC_LIGHT_RED_BRAKING"

    def __init__(
        self,
        red_brake=TRAFFIC_LIGHT_RED_BRAKE,
        hold_brake=TRAFFIC_LIGHT_HOLD_BRAKE,
        hold_speed_kmh=TRAFFIC_LIGHT_HOLD_SPEED_KMH
    ):
        self.red_brake = red_brake
        self.hold_brake = hold_brake
        self.hold_speed_kmh = hold_speed_kmh

    def inspect(self, vehicle):
        traffic_light = vehicle.get_traffic_light()

        if traffic_light is None:
            return self.information()

        light_state = self._state_name(
            traffic_light.get_state()
        )

        safety_state = self.CLEAR

        if light_state == self.RED:
            safety_state = self.RED_BRAKING
        elif light_state == self.YELLOW:
            safety_state = self.YELLOW_WARNING

        actor_id = getattr(traffic_light, "id", None)

        return self.information(
            light_state=light_state,
            safety_state=safety_state,
            distance_m=self._distance_to_trigger(
                vehicle,
                traffic_light
            ),
            actor_id=actor_id,
            event_key=(actor_id, light_state)
        )

    def apply(
        self,
        requested_control,
        traffic_light_information,
        speed_kmh
    ):
        if traffic_light_information["light_state"] != self.RED:
            return requested_control

        brake_amount = (
            self.hold_brake
            if speed_kmh <= self.hold_speed_kmh
            else self.red_brake
        )

        return self._copy_control(
            requested_control,
            throttle=0.0,
            brake=max(
                requested_control.brake,
                brake_amount
            )
        )

    @classmethod
    def information(
        cls,
        light_state=None,
        safety_state=CLEAR,
        distance_m=None,
        actor_id=None,
        event_key=None
    ):
        return {
            "light_state": light_state or cls.NONE,
            "safety_state": safety_state,
            "distance_m": distance_m,
            "actor_id": actor_id,
            "event_key": event_key
        }

    @classmethod
    def _state_name(cls, raw_state):
        state_name = getattr(raw_state, "name", None)

        if state_name is None:
            state_name = str(raw_state).split(".")[-1]

        normalized_state = state_name.upper()

        known_states = {
            cls.RED,
            cls.YELLOW,
            cls.GREEN,
            cls.OFF
        }

        if normalized_state in known_states:
            return normalized_state

        return cls.UNKNOWN

    @staticmethod
    def _distance_to_trigger(vehicle, traffic_light):
        vehicle_location = vehicle.get_location()

        try:
            trigger_location = (
                traffic_light.get_transform().transform(
                    traffic_light.trigger_volume.location
                )
            )
        except (AttributeError, RuntimeError):
            trigger_location = (
                traffic_light.get_transform().location
            )

        return math.sqrt(
            (vehicle_location.x - trigger_location.x) ** 2
            + (vehicle_location.y - trigger_location.y) ** 2
            + (vehicle_location.z - trigger_location.z) ** 2
        )

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
