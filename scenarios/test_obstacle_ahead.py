import unittest
from types import SimpleNamespace

from scenarios.obstacle_ahead import ObstacleAheadScenario


class FakeBlueprint:
    def __init__(self, blueprint_id):
        self.id = blueprint_id
        self.attributes = {}

    def has_attribute(self, name):
        return name == "role_name"

    def set_attribute(self, name, value):
        self.attributes[name] = value


class FakeBlueprintLibrary:
    def __init__(self):
        self.blueprint = FakeBlueprint("vehicle.audi.tt")

    def find(self, blueprint_id):
        if blueprint_id != "vehicle.audi.tt":
            raise IndexError(blueprint_id)
        return self.blueprint

    def filter(self, _pattern):
        return [self.blueprint]


class FakeVehicle:
    def __init__(self):
        self.control = SimpleNamespace(
            throttle=0.2,
            steer=0.1,
            brake=0.0,
            hand_brake=False
        )
        self.autopilot = None
        self.physics_enabled = True
        self.transform = None

    def get_location(self):
        if self.transform is not None:
            return self.transform.location
        return SimpleNamespace(x=0.0, y=0.0, z=0.0)

    def get_transform(self):
        if self.transform is not None:
            return self.transform
        return SimpleNamespace(
            location=self.get_location(),
            rotation=SimpleNamespace(yaw=0.0)
        )

    def get_velocity(self):
        return getattr(
            self,
            "velocity",
            SimpleNamespace(x=0.0, y=0.0, z=0.0)
        )

    def enable_constant_velocity(self, velocity):
        self.velocity = velocity

    def disable_constant_velocity(self):
        self.velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)

    def set_target_velocity(self, velocity):
        self.velocity = velocity

    def set_autopilot(self, enabled):
        self.autopilot = enabled

    def get_control(self):
        return self.control

    def apply_control(self, control):
        self.control = control

    def set_simulate_physics(self, enabled):
        self.physics_enabled = enabled

    def set_transform(self, transform):
        self.transform = transform


class FakeWaypoint:
    def __init__(self, distance=0.0):
        self.distance = distance
        self.transform = SimpleNamespace(
            location=SimpleNamespace(
                x=distance,
                y=0.0,
                z=0.0
            ),
            rotation=SimpleNamespace(yaw=0.0)
        )
        self.is_junction = False

    def next(self, distance):
        return [FakeWaypoint(self.distance + distance)]


class FakeWorld:
    def __init__(self, can_spawn=True):
        self.can_spawn = can_spawn
        self.spawned_vehicle = None
        self.blueprints = FakeBlueprintLibrary()
        self.world_map = SimpleNamespace(
            get_waypoint=lambda *_args, **_kwargs: FakeWaypoint(),
            get_spawn_points=lambda: [
                SimpleNamespace(
                    location=SimpleNamespace(x=0.0, y=0.0, z=0.0)
                )
            ]
        )

    def get_map(self):
        return self.world_map

    def get_blueprint_library(self):
        return self.blueprints

    def try_spawn_actor(self, _blueprint, transform):
        if not self.can_spawn:
            return None
        self.spawned_vehicle = FakeVehicle()
        self.spawned_vehicle.transform = transform
        return self.spawned_vehicle


class ObstacleAheadScenarioTests(unittest.TestCase):
    def test_stationary_vehicle_is_spawned_ahead(self):
        world = FakeWorld()

        actors = ObstacleAheadScenario().setup(
            world,
            FakeVehicle()
        )

        self.assertEqual(actors, [world.spawned_vehicle])
        self.assertFalse(world.spawned_vehicle.autopilot)
        self.assertEqual(world.spawned_vehicle.control.throttle, 0.0)
        self.assertEqual(world.spawned_vehicle.control.brake, 1.0)
        self.assertTrue(world.spawned_vehicle.control.hand_brake)
        self.assertFalse(world.spawned_vehicle.physics_enabled)
        self.assertEqual(world.spawned_vehicle.transform.location.x, 30.0)

    def test_setup_fails_if_no_spawn_point_is_available(self):
        with self.assertRaisesRegex(RuntimeError, "could not place"):
            ObstacleAheadScenario().setup(
                FakeWorld(can_spawn=False),
                FakeVehicle()
            )


if __name__ == "__main__":
    unittest.main()
