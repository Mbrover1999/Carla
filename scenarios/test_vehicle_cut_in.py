import unittest
from types import SimpleNamespace

from scenarios.vehicle_cut_in import VehicleCutInScenario


class Waypoint:
    def __init__(self, x, y, lane_id):
        self.lane_id = lane_id
        self.lane_type = SimpleNamespace(name="Driving")
        self.is_junction = False
        self.transform = SimpleNamespace(
            location=SimpleNamespace(x=x, y=y, z=0.0),
            rotation=SimpleNamespace(yaw=0.0)
        )
        self.left_lane = None
        self.right_lane = None

    def next(self, distance):
        result = Waypoint(
            self.transform.location.x + distance,
            self.transform.location.y,
            self.lane_id
        )
        if self.lane_id == 2:
            result.right_lane = Waypoint(
                result.transform.location.x,
                0.0,
                1
            )
        return [result]

    def get_left_lane(self):
        return self.left_lane

    def get_right_lane(self):
        return self.right_lane


class VehicleCutInScenarioTests(unittest.TestCase):
    def test_merge_route_moves_from_left_lane_to_ego_lane(self):
        scenario = VehicleCutInScenario()
        left = Waypoint(18.0, -3.5, 2)
        left.right_lane = Waypoint(18.0, 0.0, 1)

        route = scenario._build_merge_route(left)

        self.assertGreaterEqual(len(route), 3)
        self.assertEqual(route[0].lane_id, 2)
        self.assertEqual(route[2].lane_id, 1)
        self.assertEqual(route[2].transform.location.y, 0.0)

    def test_opposite_direction_left_lane_is_rejected(self):
        ego = Waypoint(0.0, 0.0, 1)
        ego.left_lane = Waypoint(0.0, -3.5, -1)

        self.assertIsNone(VehicleCutInScenario._left_driving_lane(ego))

    def test_cut_in_waits_until_ego_is_close_and_moving(self):
        scenario = VehicleCutInScenario()
        scenario.ego_vehicle = SimpleNamespace(
            get_transform=lambda: SimpleNamespace(
                location=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                rotation=SimpleNamespace(yaw=0.0)
            ),
            get_velocity=lambda: SimpleNamespace(x=5.0, y=0.0, z=0.0)
        )
        scenario.vehicle = SimpleNamespace(
            get_location=lambda: SimpleNamespace(x=10.0, y=-3.5, z=0.0)
        )

        self.assertTrue(scenario._ego_is_in_cut_in_position())

        scenario.vehicle = SimpleNamespace(
            get_location=lambda: SimpleNamespace(x=18.0, y=-3.5, z=0.0)
        )

        self.assertFalse(scenario._ego_is_in_cut_in_position())


if __name__ == "__main__":
    unittest.main()
