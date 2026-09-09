import unittest

from safety.safety_layer import SafetyLayer


class FakeControl:
    def __init__(
        self,
        throttle=0.3,
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


class SafetyLayerTests(unittest.TestCase):
    def setUp(self):
        self.layer = SafetyLayer()

    def test_low_speed_vehicle_creeps_at_normal_braking_distance(self):
        control, state = self.layer.apply(
            FakeControl(),
            obstacle_distance=4.5,
            speed_kmh=0.0
        )

        self.assertEqual(state, "CREEPING")
        self.assertEqual(control.brake, 0.0)
        self.assertLessEqual(control.throttle, 0.05)

    def test_low_speed_vehicle_still_brakes_at_stopping_distance(self):
        control, state = self.layer.apply(
            FakeControl(),
            obstacle_distance=2.7,
            speed_kmh=0.0
        )

        self.assertEqual(state, "BRAKING")
        self.assertGreater(control.brake, 0.0)

    def test_emergency_distance_is_preserved_at_low_speed(self):
        control, state = self.layer.apply(
            FakeControl(),
            obstacle_distance=2.0,
            speed_kmh=0.0
        )

        self.assertEqual(state, "EMERGENCY")
        self.assertEqual(control.brake, 1.0)

    def test_moving_vehicle_uses_normal_braking_threshold(self):
        control, state = self.layer.apply(
            FakeControl(),
            obstacle_distance=4.5,
            speed_kmh=10.0
        )

        self.assertEqual(state, "BRAKING")
        self.assertGreater(control.brake, 0.0)


if __name__ == "__main__":
    unittest.main()
