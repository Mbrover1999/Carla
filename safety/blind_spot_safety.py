import math

from config import (
    BLIND_SPOT_FORWARD_METERS,
    BLIND_SPOT_MAX_LATERAL_METERS,
    BLIND_SPOT_MIN_LATERAL_METERS,
    BLIND_SPOT_REAR_METERS
)


class BlindSpotSafety:
    CLEAR = "CLEAR"
    LEFT = "LEFT OCCUPIED"
    RIGHT = "RIGHT OCCUPIED"
    BOTH = "BOTH OCCUPIED"

    def inspect(self, world, ego_vehicle, radar_readings=None):
        left = self._nearest_vehicle(world, ego_vehicle, side="left")
        right = self._nearest_vehicle(world, ego_vehicle, side="right")

        if left is not None and right is not None:
            state = self.BOTH
        elif left is not None:
            state = self.LEFT
        elif right is not None:
            state = self.RIGHT
        else:
            state = self.CLEAR

        actor_ids = tuple(
            actor_id
            for actor_id in (
                left[1] if left is not None else None,
                right[1] if right is not None else None
            )
            if actor_id is not None
        )
        return {
            "state": state,
            "left_occupied": left is not None,
            "right_occupied": right is not None,
            "left_distance_m": left[0] if left is not None else None,
            "right_distance_m": right[0] if right is not None else None,
            "radar_readings": radar_readings or {},
            "event_key": ("blind_spot",) + actor_ids if actor_ids else None
        }

    @staticmethod
    def information():
        return {
            "state": BlindSpotSafety.CLEAR,
            "left_occupied": False,
            "right_occupied": False,
            "left_distance_m": None,
            "right_distance_m": None,
            "radar_readings": {},
            "event_key": None
        }

    def _nearest_vehicle(self, world, ego_vehicle, side):
        if world is None:
            return None

        ego_transform = ego_vehicle.get_transform()
        ego_location = ego_transform.location
        yaw = math.radians(ego_transform.rotation.yaw)
        forward = (math.cos(yaw), math.sin(yaw))
        right = (-math.sin(yaw), math.cos(yaw))
        matches = []

        try:
            actors = world.get_actors()
            actors = actors.filter("vehicle.*") if hasattr(actors, "filter") else actors
        except (AttributeError, RuntimeError):
            return None

        for actor in actors:
            if getattr(actor, "id", None) == getattr(ego_vehicle, "id", None):
                continue

            actor_type = getattr(actor, "type_id", "vehicle.unknown") or ""
            if not actor_type.startswith("vehicle."):
                continue

            try:
                location = actor.get_location()
            except (AttributeError, RuntimeError):
                continue

            offset_x = location.x - ego_location.x
            offset_y = location.y - ego_location.y
            longitudinal = offset_x * forward[0] + offset_y * forward[1]
            lateral = offset_x * right[0] + offset_y * right[1]
            side_lateral = -lateral if side == "left" else lateral

            if (
                -BLIND_SPOT_REAR_METERS <= longitudinal <= BLIND_SPOT_FORWARD_METERS
                and BLIND_SPOT_MIN_LATERAL_METERS
                <= side_lateral <= BLIND_SPOT_MAX_LATERAL_METERS
            ):
                matches.append((
                    math.hypot(offset_x, offset_y),
                    getattr(actor, "id", None)
                ))

        return min(matches, default=None, key=lambda item: item[0])
