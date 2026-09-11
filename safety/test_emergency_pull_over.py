import unittest
from types import SimpleNamespace

from safety.emergency_pull_over import EmergencyPullOverController


def location(x=0.0, y=0.0):
    return SimpleNamespace(x=x, y=y, z=0.0)


class FakeControl:
    def __init__(
        self,
        throttle=0.3,
        steer=0.0,
        brake=0.0,
        hand_brake=False,
        reverse=False,
        manual_gear_shift=False,
        gear=0
    ):
        self.throttle = throttle
        self.steer = steer
        self.brake = brake
        self.hand_brake = hand_brake
        self.reverse = reverse
        self.manual_gear_shift = manual_gear_shift
        self.gear = gear


class FakeWaypoint:
    def __init__(self, lane_id, y, lane_type="Driving"):
        self.lane_id = lane_id
        self.road_id = 7
        self.lane_type = SimpleNamespace(name=lane_type)
        self.transform = SimpleNamespace(
            location=location(y=y),
            rotation=SimpleNamespace(yaw=0.0)
        )
        self.right_lane = None

    def get_right_lane(self):
        return self.right_lane

    def next(self, distance):
        future = FakeWaypoint(
            self.lane_id,
            self.transform.location.y,
            self.lane_type.name
        )
        future.transform.location.x = distance
        return [future]


class FakeMap:
    def __init__(self, waypoint):
        self.waypoint = waypoint

    def get_waypoint(self, target, project_to_road=True):
        return self.waypoint


class FakeVehicle:
    def __init__(self, vehicle_location=None):
        self.id = 1
        self.location = vehicle_location or location()

    def get_location(self):
        return self.location

    def get_transform(self):
        return SimpleNamespace(
            location=self.location,
            rotation=SimpleNamespace(yaw=0.0)
        )


class EmergencyPullOverControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = EmergencyPullOverController(
            horn_interval_seconds=3.0,
            call_delay_seconds=120.0
        )
        self.vehicle = FakeVehicle()

    def test_moves_toward_legal_right_lane_with_hazards(self):
        current = FakeWaypoint(1, 0.0)
        current.right_lane = FakeWaypoint(2, 3.5)

        control, information = self.controller.apply(
            self.vehicle,
            FakeControl(),
            FakeMap(current),
            speed_kmh=10.0,
            inactive_seconds=5.0,
            now=10.0
        )

        self.assertGreater(control.steer, 0.0)
        self.assertEqual(
            information["phase"],
            self.controller.MOVING_RIGHT
        )
        self.assertTrue(information["hazards_active"])
        self.assertTrue(information["horn_requested"])

    def test_stops_if_no_legal_right_lane_exists(self):
        current = FakeWaypoint(1, 0.0)

        control, information = self.controller.apply(
            self.vehicle,
            FakeControl(),
            FakeMap(current),
            speed_kmh=10.0,
            inactive_seconds=5.0,
            now=10.0
        )

        self.assertEqual(control.throttle, 0.0)
        self.assertGreaterEqual(control.brake, 0.45)
        self.assertEqual(
            information["phase"],
            self.controller.STOPPING
        )

    def test_never_enters_opposite_direction_driving_lane(self):
        current = FakeWaypoint(1, 0.0)
        current.right_lane = FakeWaypoint(-1, 3.5)

        control, information = self.controller.apply(
            self.vehicle,
            FakeControl(),
            FakeMap(current),
            speed_kmh=10.0,
            inactive_seconds=5.0,
            now=10.0
        )

        self.assertEqual(control.steer, 0.0)
        self.assertEqual(
            information["phase"],
            self.controller.STOPPING
        )

    def test_existing_safety_brake_is_preserved(self):
        current = FakeWaypoint(1, 0.0)
        current.right_lane = FakeWaypoint(2, 3.5)

        control, _ = self.controller.apply(
            self.vehicle,
            FakeControl(brake=0.8),
            FakeMap(current),
            speed_kmh=10.0,
            inactive_seconds=5.0,
            now=10.0
        )

        self.assertEqual(control.throttle, 0.0)
        self.assertEqual(control.brake, 0.8)

    def test_occupied_right_lane_is_not_entered(self):
        current = FakeWaypoint(1, 0.0)
        target = FakeWaypoint(2, 3.5)
        current.right_lane = target
        blocking_vehicle = SimpleNamespace(
            id=2,
            get_location=lambda: location(x=3.0, y=3.5)
        )
        world = SimpleNamespace(
            get_actors=lambda: [blocking_vehicle]
        )
        world_map = FakeMap(current)

        def actor_waypoint(target_location, project_to_road=True):
            if target_location.y == 3.5:
                return target
            return current

        world_map.get_waypoint = actor_waypoint

        control, information = self.controller.apply(
            self.vehicle,
            FakeControl(),
            world_map,
            speed_kmh=10.0,
            inactive_seconds=5.0,
            now=10.0,
            world=world
        )

        self.assertEqual(control.steer, 0.0)
        self.assertEqual(control.brake, 1.0)
        self.assertEqual(
            information["phase"],
            self.controller.WAITING_FOR_RIGHT_LANE
        )

    def test_vehicle_inside_diagonal_merge_path_blocks_turn(self):
        current = FakeWaypoint(1, 0.0)
        target = FakeWaypoint(2, 3.5)
        current.right_lane = target
        blocking_vehicle = SimpleNamespace(
            id=2,
            get_location=lambda: location(x=0.0, y=1.8)
        )
        world = SimpleNamespace(get_actors=lambda: [blocking_vehicle])

        control, information = self.controller.apply(
            self.vehicle,
            FakeControl(),
            FakeMap(current),
            speed_kmh=10.0,
            inactive_seconds=5.0,
            now=10.0,
            world=world
        )

        self.assertEqual(control.steer, 0.0)
        self.assertEqual(control.brake, 1.0)
        self.assertEqual(
            information["phase"],
            self.controller.WAITING_FOR_RIGHT_LANE
        )

    def test_vehicle_farther_ahead_in_shoulder_corridor_blocks_merge(self):
        current = FakeWaypoint(1, 0.0)
        shoulder = FakeWaypoint(2, 3.5, lane_type="Shoulder")
        current.right_lane = shoulder
        blocking_vehicle = SimpleNamespace(
            id=2,
            get_location=lambda: location(x=22.0, y=3.5)
        )
        world = SimpleNamespace(get_actors=lambda: [blocking_vehicle])
        world_map = FakeMap(current)

        control, information = self.controller.apply(
            self.vehicle,
            FakeControl(),
            world_map,
            speed_kmh=10.0,
            inactive_seconds=5.0,
            now=10.0,
            world=world
        )

        self.assertEqual(control.steer, 0.0)
        self.assertGreater(control.brake, 0.0)
        self.assertEqual(
            information["phase"],
            self.controller.WAITING_FOR_RIGHT_LANE
        )

    def test_brakes_immediately_after_entering_shoulder(self):
        current = FakeWaypoint(1, 0.0)
        shoulder = FakeWaypoint(2, 3.5, lane_type="Shoulder")
        # Even if CARLA exposes another lane farther right, the shoulder
        # must remain the final destination.
        shoulder.right_lane = FakeWaypoint(
            3,
            5.0,
            lane_type="Parking"
        )
        current.right_lane = shoulder
        vehicle = FakeVehicle(location(y=2.0))

        control, information = self.controller.apply(
            vehicle,
            FakeControl(throttle=0.3),
            FakeMap(current),
            speed_kmh=10.0,
            inactive_seconds=5.0,
            now=10.0
        )

        self.assertEqual(control.throttle, 0.0)
        self.assertGreaterEqual(control.brake, 0.45)
        self.assertEqual(
            information["phase"],
            self.controller.STOPPING
        )

    def test_simulated_call_is_requested_once_after_two_minutes(self):
        current = FakeWaypoint(1, 0.0)
        world_map = FakeMap(current)

        _, first = self.controller.apply(
            self.vehicle,
            FakeControl(),
            world_map,
            speed_kmh=0.0,
            inactive_seconds=120.0,
            now=120.0
        )
        _, second = self.controller.apply(
            self.vehicle,
            FakeControl(),
            world_map,
            speed_kmh=0.0,
            inactive_seconds=121.0,
            now=121.0
        )

        self.assertTrue(first["call_requested"])
        self.assertTrue(first["call_started"])
        self.assertFalse(second["call_requested"])

    def test_wakeup_alarm_repeats_even_after_vehicle_stops(self):
        current = FakeWaypoint(1, 0.0)
        world_map = FakeMap(current)

        _, first = self.controller.apply(
            self.vehicle,
            FakeControl(),
            world_map,
            speed_kmh=0.0,
            inactive_seconds=5.0,
            now=10.0
        )
        _, too_soon = self.controller.apply(
            self.vehicle,
            FakeControl(),
            world_map,
            speed_kmh=0.0,
            inactive_seconds=6.0,
            now=11.0
        )
        _, repeated = self.controller.apply(
            self.vehicle,
            FakeControl(),
            world_map,
            speed_kmh=0.0,
            inactive_seconds=8.1,
            now=13.1
        )

        self.assertEqual(first["phase"], self.controller.STOPPED)
        self.assertTrue(first["horn_requested"])
        self.assertFalse(too_soon["horn_requested"])
        self.assertTrue(repeated["horn_requested"])


if __name__ == "__main__":
    unittest.main()
