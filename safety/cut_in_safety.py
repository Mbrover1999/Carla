import math
import time

from config import (
    CUT_IN_BRAKE_AMOUNT,
    CUT_IN_BRAKE_TIME_SECONDS,
    CUT_IN_CORRIDOR_MARGIN_METERS,
    CUT_IN_DEPARTURE_CONFIRMATION_SECONDS,
    CUT_IN_EMERGENCY_BRAKE,
    CUT_IN_EMERGENCY_LONGITUDINAL_METERS,
    CUT_IN_EMERGENCY_TIME_SECONDS,
    CUT_IN_FORWARD_RANGE_METERS,
    CUT_IN_INTERVENTION_HOLD_SECONDS,
    CUT_IN_MIN_HEADING_ALIGNMENT,
    CUT_IN_MIN_LANE_DEPARTURE_METERS,
    CUT_IN_MIN_LATERAL_APPROACH_MPS,
    CUT_IN_MIN_PREDICTED_DEPARTURE_METERS,
    CUT_IN_MAX_FORWARD_GAP_SECONDS,
    CUT_IN_MIN_FORWARD_GAP_METERS,
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

    def __init__(self, clock=None):
        self.clock = clock or time.monotonic
        self.latched_information = None
        self.latched_until = 0.0

    def inspect(self, world, world_map, ego_vehicle, active=True):
        if not active or world is None:
            self._clear_latch()
            return self.information(safety_state=self.DISABLED)

        ego_transform = ego_vehicle.get_transform()
        ego_location = ego_transform.location
        yaw = math.radians(ego_transform.rotation.yaw)
        forward = (math.cos(yaw), math.sin(yaw))
        right = (-math.sin(yaw), math.cos(yaw))
        ego_velocity = ego_vehicle.get_velocity()
        ego_forward_speed = self._dot_velocity(ego_velocity, forward)
        ego_half_width, ego_half_length = self._vehicle_extents(
            ego_vehicle
        )
        try:
            ego_waypoint = world_map.get_waypoint(
                ego_location,
                project_to_road=True
            )
        except (AttributeError, RuntimeError):
            ego_waypoint = None

        if ego_waypoint is None:
            return self._latched_or_clear()

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
                    world_map=world_map,
                    ego_waypoint=ego_waypoint,
                    ego_location=ego_location,
                    forward=forward,
                    right=right,
                    ego_forward_speed=ego_forward_speed,
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
            return self._latched_or_clear()

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

        information = self.information(
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

        if state in (self.BRAKING, self.EMERGENCY):
            self.latched_information = information
            self.latched_until = (
                self.clock() + CUT_IN_INTERVENTION_HOLD_SECONDS
            )
        elif self.clock() < self.latched_until:
            return dict(self.latched_information)

        return information

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
        world_map,
        ego_waypoint,
        ego_location,
        forward,
        right,
        ego_forward_speed,
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

        actor_waypoint = world_map.get_waypoint(
            actor_location,
            project_to_road=True
        )

        if actor_waypoint is None:
            return None

        if (
            getattr(ego_waypoint, "is_junction", False)
            or getattr(actor_waypoint, "is_junction", False)
        ):
            return None

        ego_road = getattr(ego_waypoint, "road_id", None)
        actor_road = getattr(actor_waypoint, "road_id", None)
        ego_lane = getattr(ego_waypoint, "lane_id", 0)
        actor_lane = getattr(actor_waypoint, "lane_id", 0)

        # Cut-in prediction is only for a neighbouring, same-direction lane
        # on the same road. Junction conflicts are handled separately by the
        # cross-traffic safety layer.
        if (
            ego_road != actor_road
            or ego_lane == actor_lane
            or ego_lane == 0
            or actor_lane == 0
            or ego_lane * actor_lane < 0
        ):
            return None

        actor_velocity = actor.get_velocity()
        actor_forward_speed = self._dot_velocity(actor_velocity, forward)
        lane_yaw = math.radians(
            actor_waypoint.transform.rotation.yaw
        )
        actor_lane_right = (-math.sin(lane_yaw), math.cos(lane_yaw))
        actor_lateral_speed = self._dot_velocity(
            actor_velocity,
            actor_lane_right
        )
        actor_to_ego_x = ego_location.x - actor_location.x
        actor_to_ego_y = ego_location.y - actor_location.y
        ego_side_in_actor_lane = (
            actor_to_ego_x * actor_lane_right[0]
            + actor_to_ego_y * actor_lane_right[1]
        )
        lateral_approach_speed = (
            math.copysign(1.0, ego_side_in_actor_lane)
            * actor_lateral_speed
            if abs(ego_side_in_actor_lane) > 0.01
            else 0.0
        )

        # A car following a bend has almost no velocity across its own local
        # lane. A genuine lane change does, so it is safe to reject anything
        # below this threshold without widening the forward obstacle sensor.
        if lateral_approach_speed < CUT_IN_MIN_LATERAL_APPROACH_MPS:
            return None

        lane_centre = actor_waypoint.transform.location
        lane_offset_x = actor_location.x - lane_centre.x
        lane_offset_y = actor_location.y - lane_centre.y
        lane_offset_toward_ego = (
            math.copysign(1.0, ego_side_in_actor_lane)
            * (
                lane_offset_x * actor_lane_right[0]
                + lane_offset_y * actor_lane_right[1]
            )
        )
        predicted_lane_departure = (
            lane_offset_toward_ego
            + lateral_approach_speed
            * CUT_IN_DEPARTURE_CONFIRMATION_SECONDS
        )

        # Curved-road tracking can produce a temporary lateral velocity even
        # while the actor is centred in its own lane. A genuine cut-in must
        # already be leaving that centre line and continue far enough toward
        # the ego lane during the short confirmation horizon.
        if (
            lane_offset_toward_ego
            < CUT_IN_MIN_LANE_DEPARTURE_METERS
            or predicted_lane_departure
            < CUT_IN_MIN_PREDICTED_DEPARTURE_METERS
        ):
            return None

        if self._heading_alignment(actor, forward) < (
            CUT_IN_MIN_HEADING_ALIGNMENT
        ):
            return None

        actor_half_width, actor_half_length = self._vehicle_extents(actor)
        lateral_clearance = (
            abs(ego_side_in_actor_lane)
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

        forward_bumper_gap = (
            predicted_longitudinal
            - ego_half_length
            - actor_half_length
        )
        maximum_relevant_gap = max(
            CUT_IN_MIN_FORWARD_GAP_METERS,
            max(0.0, ego_forward_speed)
            * CUT_IN_MAX_FORWARD_GAP_SECONDS
        )

        # A lane change far ahead is not an immediate cut-in hazard. The
        # normal forward obstacle layer will manage following distance after
        # the vehicle completes its merge.
        if forward_bumper_gap > maximum_relevant_gap:
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

    def _latched_or_clear(self):
        if (
            self.latched_information is not None
            and self.clock() < self.latched_until
        ):
            return dict(self.latched_information)

        self._clear_latch()
        return self.information()

    def _clear_latch(self):
        self.latched_information = None
        self.latched_until = 0.0

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
