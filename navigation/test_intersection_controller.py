import unittest
from types import SimpleNamespace

from navigation.intersection_controller import (
    IntersectionController
)


class FakeControl:
    def __init__(
        self,
        throttle=0.2,
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


def make_transform(x=0.0, y=0.0, yaw=0.0):
    return SimpleNamespace(
        location=SimpleNamespace(x=x, y=y, z=0.0),
        rotation=SimpleNamespace(yaw=yaw)
    )


class FakeVehicle:
    def __init__(self, transform):
        self.transform = transform

    def get_transform(self):
        return self.transform


class IntersectionControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = IntersectionController()
        self.vehicle = FakeVehicle(make_transform())

    def test_target_on_left_produces_left_steering(self):
        target = SimpleNamespace(
            transform=make_transform(x=5.0, y=-5.0)
        )

        control, information = self.controller.apply(
            self.vehicle,
            FakeControl(),
            target
        )

        self.assertLess(control.steer, 0.0)
        self.assertTrue(information["active"])

    def test_target_on_right_produces_right_steering(self):
        target = SimpleNamespace(
            transform=make_transform(x=5.0, y=5.0)
        )

        control, _ = self.controller.apply(
            self.vehicle,
            FakeControl(),
            target
        )

        self.assertGreater(control.steer, 0.0)

    def test_missing_target_does_not_modify_control(self):
        requested_control = FakeControl(steer=0.2)

        control, information = self.controller.apply(
            self.vehicle,
            requested_control,
            None
        )

        self.assertIs(control, requested_control)
        self.assertFalse(information["active"])


if __name__ == "__main__":
    unittest.main()
