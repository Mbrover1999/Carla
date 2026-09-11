import math

from config import (
    TRAFFIC_LIGHT_BRAKING_TIME_SECONDS,
    TRAFFIC_LIGHT_CREEP_MAX_SPEED_KMH,
    TRAFFIC_LIGHT_CREEP_THROTTLE,
    TRAFFIC_LIGHT_HOLD_BRAKE,
    TRAFFIC_LIGHT_HOLD_SPEED_KMH,
    TRAFFIC_LIGHT_MIN_BRAKING_RANGE_METERS,
    TRAFFIC_LIGHT_RED_BRAKE,
    TRAFFIC_LIGHT_STOP_DISTANCE_METERS
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
    RED_CREEPING = "TRAFFIC_LIGHT_RED_CREEPING"
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
        self.tracked_traffic_light = None
        self.tracked_actor_id = None
        self.red_hold_latched = False

    def inspect(self, vehicle):
        traffic_light = vehicle.get_traffic_light()

        if traffic_light is None:
            traffic_light = self.tracked_traffic_light

        if traffic_light is None:
            return self.information()

        actor_id = getattr(traffic_light, "id", None)

        if actor_id != self.tracked_actor_id:
            self.tracked_traffic_light = traffic_light
            self.tracked_actor_id = actor_id
            self.red_hold_latched = False

        try:
            light_state = self._state_name(
                traffic_light.get_state()
            )
        except (AttributeError, RuntimeError):
            self.reset()
            return self.information()

        if light_state != self.RED:
            self.reset()

        safety_state = self.CLEAR

        if light_state == self.YELLOW:
            safety_state = self.YELLOW_WARNING

        return self.information(
            light_state=light_state,
            safety_state=safety_state,
            distance_m=self._distance_to_stop_line(
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
            self.red_hold_latched = False
            return requested_control

        distance = traffic_light_information.get("distance_m")

        if self.red_hold_latched:
            self._set_safety_state(
                traffic_light_information,
                self.RED_BRAKING
            )
            return self._copy_control(
                requested_control,
                throttle=0.0,
                brake=max(
                    requested_control.brake,
                    self.hold_brake
                )
            )

        speed_mps = speed_kmh / 3.6
        braking_range = (
            TRAFFIC_LIGHT_STOP_DISTANCE_METERS
            + max(
                TRAFFIC_LIGHT_MIN_BRAKING_RANGE_METERS,
                speed_mps * TRAFFIC_LIGHT_BRAKING_TIME_SECONDS
            )
        )

        if distance is not None and distance > braking_range:
            self._set_safety_state(
                traffic_light_information,
                self.CLEAR
            )
            return requested_control

        if (
            distance is None
            or distance <= TRAFFIC_LIGHT_STOP_DISTANCE_METERS
        ):
            # Once the vehicle reaches the stop line, keep the red-light
            # hold latched. CARLA may stop reporting get_traffic_light()
            # immediately after the trigger line, and releasing here would
            # allow the vehicle to creep into the intersection.
            self.red_hold_latched = True

        if (
            distance is not None
            and distance > TRAFFIC_LIGHT_STOP_DISTANCE_METERS
            and speed_kmh <= TRAFFIC_LIGHT_CREEP_MAX_SPEED_KMH
        ):
            self._set_safety_state(
                traffic_light_information,
                self.RED_CREEPING
            )

            # Do not release a brake requested by another safety system.
            if requested_control.brake > 0.0:
                return requested_control

            return self._copy_control(
                requested_control,
                throttle=min(
                    requested_control.throttle,
                    TRAFFIC_LIGHT_CREEP_THROTTLE
                ),
                brake=0.0
            )

        self._set_safety_state(
            traffic_light_information,
            self.RED_BRAKING
        )

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

    def reset(self):
        self.tracked_traffic_light = None
        self.tracked_actor_id = None
        self.red_hold_latched = False

    @staticmethod
    def _set_safety_state(information, safety_state):
        information["safety_state"] = safety_state
        information["event_key"] = (
            information.get("actor_id"),
            information.get("light_state"),
            safety_state
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
    def _distance_to_stop_line(vehicle, traffic_light):
        vehicle_location = vehicle.get_location()

        try:
            stop_waypoints = traffic_light.get_stop_waypoints()
        except (AttributeError, RuntimeError):
            stop_waypoints = None

        if stop_waypoints:
            stop_locations = [
                waypoint.transform.location
                for waypoint in stop_waypoints
            ]

            return min(
                math.sqrt(
                    (vehicle_location.x - location.x) ** 2
                    + (vehicle_location.y - location.y) ** 2
                    + (vehicle_location.z - location.z) ** 2
                )
                for location in stop_locations
            )

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
