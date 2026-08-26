import unittest

from navigation.road_speed_controller import RoadSpeedController


class FakeControl:
    def __init__(
        self,
        throttle=0.0,
        steer=0.1,
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


class FakeVehicle:
    def __init__(self, speed_limit):
        self.speed_limit = speed_limit

    def get_speed_limit(self):
        return self.speed_limit


class RoadSpeedControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = RoadSpeedController()

    def test_uses_road_speed_limit_with_safety_margin(self):
        _, information = self.controller.apply(
            FakeVehicle(50.0),
            FakeControl(),
            current_speed_kmh=30.0,
            navigation_mode="AI"
        )

        self.assertEqual(information["speed_limit_kmh"], 50.0)
        self.assertEqual(information["target_speed_kmh"], 45.0)

    def test_accelerates_below_target_speed(self):
        control, _ = self.controller.apply(
            FakeVehicle(30.0),
            FakeControl(),
            current_speed_kmh=10.0,
            navigation_mode="AI"
        )

        self.assertGreater(control.throttle, 0.0)
        self.assertEqual(control.brake, 0.0)

    def test_brakes_above_target_speed(self):
        control, _ = self.controller.apply(
            FakeVehicle(30.0),
            FakeControl(),
            current_speed_kmh=40.0,
            navigation_mode="AI"
        )

        self.assertEqual(control.throttle, 0.0)
        self.assertGreater(control.brake, 0.0)

    def test_reduces_target_speed_for_intersection(self):
        _, information = self.controller.apply(
            FakeVehicle(50.0),
            FakeControl(),
            current_speed_kmh=30.0,
            navigation_mode="APPROACH"
        )

        self.assertEqual(information["target_speed_kmh"], 16.0)

    def test_keeps_last_valid_limit_after_spawn_gap(self):
        self.controller.apply(
            FakeVehicle(40.0),
            FakeControl(),
            current_speed_kmh=20.0,
            navigation_mode="AI"
        )

        _, information = self.controller.apply(
            FakeVehicle(0.0),
            FakeControl(),
            current_speed_kmh=20.0,
            navigation_mode="AI"
        )

        self.assertEqual(information["speed_limit_kmh"], 40.0)


if __name__ == "__main__":
    unittest.main()
