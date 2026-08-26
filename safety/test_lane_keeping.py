import unittest
from types import SimpleNamespace

from safety.lane_keeping import LaneKeepingAssist


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
        location=SimpleNamespace(x=x, y=y),
        rotation=SimpleNamespace(yaw=yaw)
    )


class FakeVehicle:
    def __init__(self, transform):
        self.transform = transform

    def get_transform(self):
        return self.transform


class FakeMap:
    def __init__(self, waypoint):
        self.waypoint = waypoint

    def get_waypoint(self, location, project_to_road=True):
        return self.waypoint


class LaneKeepingAssistTests(unittest.TestCase):
    def make_assist(self, vehicle_y=0.0, vehicle_yaw=0.0):
        waypoint = SimpleNamespace(
            transform=make_transform(),
            is_junction=False
        )
        assist = LaneKeepingAssist(FakeMap(waypoint))
        vehicle = FakeVehicle(
            make_transform(y=vehicle_y, yaw=vehicle_yaw)
        )
        return assist, vehicle

    def test_centered_vehicle_is_not_modified(self):
        assist, vehicle = self.make_assist(vehicle_y=0.1)
        control = FakeControl(steer=0.05)

        result, information = assist.apply(
            vehicle,
            control,
            speed_kmh=20.0
        )

        self.assertIs(result, control)
        self.assertEqual(information["state"], assist.CENTERED)

    def test_vehicle_right_of_center_is_steered_left(self):
        assist, vehicle = self.make_assist(vehicle_y=0.8)

        result, information = assist.apply(
            vehicle,
            FakeControl(),
            speed_kmh=20.0
        )

        self.assertLess(result.steer, 0.0)
        self.assertEqual(
            information["state"],
            assist.CORRECTING_LEFT
        )

    def test_vehicle_left_of_center_is_steered_right(self):
        assist, vehicle = self.make_assist(vehicle_y=-0.8)

        result, information = assist.apply(
            vehicle,
            FakeControl(),
            speed_kmh=20.0
        )

        self.assertGreater(result.steer, 0.0)
        self.assertEqual(
            information["state"],
            assist.CORRECTING_RIGHT
        )

    def test_heading_error_triggers_correction(self):
        assist, vehicle = self.make_assist(vehicle_yaw=10.0)

        result, information = assist.apply(
            vehicle,
            FakeControl(),
            speed_kmh=20.0
        )

        self.assertLess(result.steer, 0.0)
        self.assertEqual(
            information["state"],
            assist.CORRECTING_LEFT
        )

    def test_assist_is_inactive_at_low_speed(self):
        assist, vehicle = self.make_assist(vehicle_y=1.0)
        control = FakeControl()

        result, information = assist.apply(
            vehicle,
            control,
            speed_kmh=2.0
        )

        self.assertIs(result, control)
        self.assertEqual(information["state"], assist.LOW_SPEED)

    def test_assist_is_inactive_in_junction(self):
        waypoint = SimpleNamespace(
            transform=make_transform(),
            is_junction=True
        )
        assist = LaneKeepingAssist(FakeMap(waypoint))
        control = FakeControl()

        result, information = assist.apply(
            FakeVehicle(make_transform(y=1.0)),
            control,
            speed_kmh=20.0
        )

        self.assertIs(result, control)
        self.assertEqual(information["state"], assist.JUNCTION)


if __name__ == "__main__":
    unittest.main()
