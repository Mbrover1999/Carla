import unittest
from types import SimpleNamespace

from safety.blind_spot_safety import BlindSpotSafety


def location(x=0.0, y=0.0):
    return SimpleNamespace(x=x, y=y, z=0.0)


class Vehicle:
    def __init__(self, actor_id, x=0.0, y=0.0, yaw=0.0):
        self.id = actor_id
        self.type_id = "vehicle.test"
        self._location = location(x, y)
        self._yaw = yaw

    def get_location(self):
        return self._location

    def get_transform(self):
        return SimpleNamespace(
            location=self._location,
            rotation=SimpleNamespace(yaw=self._yaw)
        )


class BlindSpotSafetyTests(unittest.TestCase):
    def inspect(self, *vehicles):
        world = SimpleNamespace(get_actors=lambda: list(vehicles))
        return BlindSpotSafety().inspect(world, vehicles[0])

    def test_detects_vehicle_in_left_rear_zone(self):
        information = self.inspect(
            Vehicle(1),
            Vehicle(2, x=-3.0, y=-3.4)
        )

        self.assertTrue(information["left_occupied"])
        self.assertFalse(information["right_occupied"])
        self.assertEqual(information["state"], BlindSpotSafety.LEFT)

    def test_detects_vehicle_in_right_side_zone(self):
        information = self.inspect(
            Vehicle(1),
            Vehicle(2, x=1.0, y=3.5)
        )

        self.assertTrue(information["right_occupied"])
        self.assertEqual(information["state"], BlindSpotSafety.RIGHT)

    def test_ignores_vehicle_well_ahead(self):
        information = self.inspect(Vehicle(1), Vehicle(2, x=15.0, y=3.5))

        self.assertEqual(information["state"], BlindSpotSafety.CLEAR)

    def test_zone_rotates_with_ego_vehicle(self):
        # At yaw 90 degrees, the ego vehicle's right side points west.
        information = self.inspect(
            Vehicle(1, yaw=90.0),
            Vehicle(2, x=-3.5, y=-2.0)
        )

        self.assertTrue(information["right_occupied"])


if __name__ == "__main__":
    unittest.main()
