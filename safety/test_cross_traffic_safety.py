import unittest
from types import SimpleNamespace

from safety.cross_traffic_safety import CrossTrafficSafety


def vector(x, y):
    return SimpleNamespace(x=x, y=y, z=0.0)


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


class FakeActor:
    def __init__(self, actor_id, x, y, vx, vy):
        self.id = actor_id
        self.type_id = "vehicle.test"
        self.is_alive = True
        self.location = vector(x, y)
        self.velocity = vector(vx, vy)

    def get_location(self):
        return self.location

    def get_velocity(self):
        return self.velocity


class FakeEgo(FakeActor):
    def get_transform(self):
        return SimpleNamespace(
            rotation=SimpleNamespace(yaw=0.0)
        )


class FakeWorld:
    def __init__(self, actors):
        self.actors = actors

    def get_actors(self):
        return self.actors


class CrossTrafficSafetyTests(unittest.TestCase):
    def setUp(self):
        self.safety = CrossTrafficSafety()
        self.ego = FakeEgo(1, 0.0, 0.0, 5.0, 0.0)

    def test_crossing_vehicle_with_matching_arrival_time_brakes(self):
        crossing = FakeActor(2, 5.0, -5.0, 0.0, 5.0)

        information = self.safety.inspect(
            FakeWorld([self.ego, crossing]),
            self.ego,
            active=True
        )

        self.assertEqual(
            information["safety_state"],
            self.safety.BRAKING
        )
        self.assertEqual(information["actor_id"], 2)

    def test_vehicle_moving_away_is_clear(self):
        crossing = FakeActor(2, 5.0, -5.0, 0.0, -5.0)

        information = self.safety.inspect(
            FakeWorld([self.ego, crossing]),
            self.ego,
            active=True
        )

        self.assertEqual(
            information["safety_state"],
            self.safety.CLEAR
        )

    def test_parallel_vehicle_is_clear(self):
        parallel = FakeActor(2, 5.0, 2.0, 5.0, 0.0)

        information = self.safety.inspect(
            FakeWorld([self.ego, parallel]),
            self.ego,
            active=True
        )

        self.assertEqual(
            information["safety_state"],
            self.safety.CLEAR
        )

    def test_detection_is_disabled_outside_intersection(self):
        crossing = FakeActor(2, 5.0, -5.0, 0.0, 5.0)

        information = self.safety.inspect(
            FakeWorld([self.ego, crossing]),
            self.ego,
            active=False
        )

        self.assertEqual(
            information["safety_state"],
            self.safety.DISABLED
        )

    def test_braking_state_overrides_throttle(self):
        information = self.safety.information(
            safety_state=self.safety.BRAKING
        )

        control = self.safety.apply(
            FakeControl(),
            information,
            speed_kmh=10.0
        )

        self.assertEqual(control.throttle, 0.0)
        self.assertEqual(control.brake, 0.75)


if __name__ == "__main__":
    unittest.main()
