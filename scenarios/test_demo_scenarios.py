import unittest
from enum import Enum
from types import SimpleNamespace

from scenarios.cross_traffic import CrossTrafficScenario
from scenarios.driver_inactivity import DriverInactivityScenario
from scenarios.lane_departure import LaneDepartureScenario
from scenarios.red_traffic_light import RedTrafficLightScenario
from scenarios.scenario_setup import ScenarioRuntime
from scenarios.test_obstacle_ahead import (
    FakeVehicle,
    FakeWaypoint,
    FakeWorld
)


class FakeLightState(Enum):
    Red = 1
    Green = 2


class DemoWaypoint(FakeWaypoint):
    def __init__(self, distance=0.0, yaw=0.0):
        super().__init__(distance)
        self.transform.rotation.yaw = yaw

    def previous(self, distance):
        return [DemoWaypoint(self.distance - distance, self.transform.rotation.yaw)]


class FakeTrafficLight:
    _next_id = 1

    def __init__(self, waypoint, state=FakeLightState.Green):
        self.id = FakeTrafficLight._next_id
        FakeTrafficLight._next_id += 1
        self.waypoint = waypoint
        self.state = state
        self.frozen = False
        self.group = [self]

    def get_stop_waypoints(self):
        return [self.waypoint]

    def get_group_traffic_lights(self):
        return self.group

    def get_state(self):
        return self.state

    def set_state(self, state):
        self.state = state

    def freeze(self, frozen):
        self.frozen = frozen


class FakeActorCollection(list):
    def filter(self, pattern):
        if pattern.startswith("traffic.traffic_light"):
            return self
        return []


class TrafficLightWorld(FakeWorld):
    def __init__(self, lights):
        super().__init__()
        self.lights = FakeActorCollection(lights)

    def get_actors(self):
        return self.lights

    def wait_for_tick(self):
        return None


class DemoScenarioTests(unittest.TestCase):
    def test_lane_departure_injects_steering_then_releases_assist(self):
        scenario = LaneDepartureScenario()
        scenario.setup(FakeWorld(), FakeVehicle())
        control = FakeVehicle().get_control()

        drift_control = scenario.apply_requested_control(4.5, control)

        self.assertEqual(drift_control.steer, scenario.DEPARTURE_STEERING)
        self.assertTrue(scenario.suppress_lane_keeping(4.5))
        self.assertIs(
            scenario.apply_requested_control(6.0, control),
            control
        )
        self.assertFalse(scenario.suppress_lane_keeping(6.0))

    def test_driver_inactivity_starts_after_initial_drive(self):
        scenario = DriverInactivityScenario()

        self.assertFalse(scenario.controller_inactive(3.9))
        self.assertTrue(scenario.controller_inactive(4.0))

    def test_red_light_is_forced_and_restored(self):
        light = FakeTrafficLight(DemoWaypoint())
        world = TrafficLightWorld([light])
        ego_vehicle = FakeVehicle()
        scenario = RedTrafficLightScenario()

        actors = scenario.setup(world, ego_vehicle)

        self.assertEqual(actors, [])
        self.assertEqual(light.state, FakeLightState.Red)
        self.assertTrue(light.frozen)
        self.assertEqual(ego_vehicle.transform.location.x, -28.0)

        scenario.close()

        self.assertEqual(light.state, FakeLightState.Green)
        self.assertFalse(light.frozen)

    def test_cross_vehicle_waits_then_enters_intersection(self):
        first_light = FakeTrafficLight(DemoWaypoint(yaw=0.0))
        second_light = FakeTrafficLight(DemoWaypoint(yaw=90.0))
        group = [first_light, second_light]
        first_light.group = group
        second_light.group = group
        world = TrafficLightWorld(group)
        scenario = CrossTrafficScenario()

        actors = scenario.setup(world, FakeVehicle())
        crossing_vehicle = actors[0]

        self.assertEqual(crossing_vehicle.control.brake, 1.0)
        scenario.update(1.9)
        self.assertEqual(crossing_vehicle.control.throttle, 0.0)
        scenario.update(2.0)
        self.assertEqual(
            crossing_vehicle.control.throttle,
            scenario.CROSSING_THROTTLE
        )
        self.assertEqual(crossing_vehicle.control.brake, 0.0)
        scenario.update(7.0)
        self.assertEqual(crossing_vehicle.control.brake, 1.0)

    def test_runtime_delegates_scenario_hooks(self):
        scenario = SimpleNamespace(
            controller_inactive=lambda elapsed: elapsed >= 4.0,
            suppress_lane_keeping=lambda elapsed: elapsed < 2.0,
            apply_requested_control=lambda _elapsed, control: control
        )
        runtime = ScenarioRuntime(scenario=scenario)
        control = object()

        self.assertTrue(runtime.controller_inactive(4.0))
        self.assertTrue(runtime.suppress_lane_keeping(1.0))
        self.assertIs(runtime.apply_requested_control(1.0, control), control)


if __name__ == "__main__":
    unittest.main()
