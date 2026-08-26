import unittest
from types import SimpleNamespace

from navigation.route_manager import (
    RouteManager,
    WaypointGraphPlanner
)


def location(x, y=0.0):
    return SimpleNamespace(x=x, y=y, z=0.0)


def waypoint(x, is_junction=False):
    return SimpleNamespace(
        transform=SimpleNamespace(location=location(x)),
        is_junction=is_junction
    )


def option(name):
    return SimpleNamespace(name=name)


class FakeVehicle:
    def __init__(self):
        self.location = location(0.0)

    def get_location(self):
        return self.location


class FakeMap:
    def get_spawn_points(self):
        return [
            SimpleNamespace(location=location(200.0))
        ]


class FakePlanner:
    def __init__(self):
        self.route = [
            (waypoint(0.0), option("LANEFOLLOW")),
            (waypoint(5.0), option("LANEFOLLOW")),
            (waypoint(10.0), option("LEFT")),
            (waypoint(15.0, is_junction=True), option("LEFT")),
            (waypoint(20.0, is_junction=True), option("LEFT")),
            (waypoint(25.0), option("LANEFOLLOW")),
            (waypoint(200.0), option("LANEFOLLOW"))
        ]

    def trace_route(self, origin, destination):
        return self.route


class RouteManagerTests(unittest.TestCase):
    def setUp(self):
        self.vehicle = FakeVehicle()
        self.manager = RouteManager(
            FakeMap(),
            self.vehicle,
            planner=FakePlanner()
        )

    def test_reports_next_maneuver(self):
        information = self.manager.update()

        self.assertEqual(information["maneuver"], "LEFT")
        self.assertEqual(information["maneuver_distance_m"], 10.0)
        self.assertEqual(information["mode"], self.manager.APPROACH)

    def test_enters_intersection_mode_on_junction_waypoint(self):
        self.vehicle.location = location(16.0)

        information = self.manager.update()

        self.assertEqual(
            information["mode"],
            self.manager.INTERSECTION
        )

    def test_provides_lookahead_waypoint(self):
        information = self.manager.update()

        self.assertGreaterEqual(
            information["target_waypoint"].transform.location.x,
            7.0
        )


class GraphWaypoint:
    def __init__(self, waypoint_id, x, is_junction=False, yaw=0.0):
        self.id = waypoint_id
        self.transform = SimpleNamespace(
            location=location(x),
            rotation=SimpleNamespace(yaw=yaw)
        )
        self.is_junction = is_junction
        self.following = []

    def next(self, distance):
        return self.following


class GraphMap:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal

    def get_waypoint(self, target, project_to_road=True):
        return self.start if target.x < 1.0 else self.goal


class WaypointGraphPlannerTests(unittest.TestCase):
    def test_fallback_planner_finds_connected_route(self):
        first = GraphWaypoint(1, 0.0)
        second = GraphWaypoint(2, 2.0, is_junction=True)
        third = GraphWaypoint(3, 4.0, is_junction=True)
        first.following = [second]
        second.following = [third]

        planner = WaypointGraphPlanner(
            GraphMap(first, third),
            sampling_resolution=2.0
        )

        route = planner.trace_route(
            location(0.0),
            location(4.0)
        )

        self.assertEqual(len(route), 3)
        self.assertEqual(route[1][1], "STRAIGHT")

    def test_fallback_classifies_turn_from_entry_to_exit(self):
        entry = GraphWaypoint(1, 0.0, yaw=0.0)
        middle = GraphWaypoint(
            2,
            2.0,
            is_junction=True,
            yaw=-20.0
        )
        exit_waypoint = GraphWaypoint(3, 4.0, yaw=-90.0)
        planner = WaypointGraphPlanner(
            GraphMap(entry, exit_waypoint),
            sampling_resolution=2.0
        )

        option_name = planner._road_option(
            [entry, middle, exit_waypoint],
            0
        )

        self.assertEqual(option_name, "LEFT")


if __name__ == "__main__":
    unittest.main()
