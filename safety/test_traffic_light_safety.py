import unittest
from types import SimpleNamespace

from safety.traffic_light_safety import TrafficLightSafety


class FakeControl:
    def __init__(
        self,
        throttle=0.2,
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


class FakeTrafficLight:
    def __init__(self, state, actor_id=42, distance=10.0):
        self.state = SimpleNamespace(name=state)
        self.id = actor_id
        self.trigger_volume = SimpleNamespace(
            location=SimpleNamespace(
                x=distance,
                y=0.0,
                z=0.0
            )
        )

    def get_state(self):
        return self.state

    def set_state(self, state):
        self.state = SimpleNamespace(name=state)

    def get_transform(self):
        return SimpleNamespace(
            location=SimpleNamespace(x=10.0, y=0.0, z=0.0),
            transform=lambda location: location
        )


class FakeVehicle:
    def __init__(self, traffic_light=None):
        self.traffic_light = traffic_light

    def get_traffic_light(self):
        return self.traffic_light

    def get_location(self):
        return SimpleNamespace(x=0.0, y=0.0, z=0.0)


class TrafficLightSafetyTests(unittest.TestCase):
    def setUp(self):
        self.safety = TrafficLightSafety()

    def test_no_traffic_light_is_clear(self):
        information = self.safety.inspect(FakeVehicle())

        self.assertEqual(information["light_state"], self.safety.NONE)
        self.assertEqual(information["safety_state"], self.safety.CLEAR)

    def test_green_light_does_not_modify_control(self):
        vehicle = FakeVehicle(FakeTrafficLight("Green"))
        information = self.safety.inspect(vehicle)
        control = FakeControl()

        result = self.safety.apply(control, information, 20.0)

        self.assertIs(result, control)
        self.assertEqual(information["light_state"], self.safety.GREEN)

    def test_yellow_light_warns_without_braking(self):
        vehicle = FakeVehicle(FakeTrafficLight("Yellow"))
        information = self.safety.inspect(vehicle)
        control = FakeControl()

        result = self.safety.apply(control, information, 20.0)

        self.assertIs(result, control)
        self.assertEqual(
            information["safety_state"],
            self.safety.YELLOW_WARNING
        )

    def test_red_light_removes_throttle_and_brakes(self):
        vehicle = FakeVehicle(
            FakeTrafficLight("Red", distance=5.0)
        )
        information = self.safety.inspect(vehicle)

        result = self.safety.apply(
            FakeControl(throttle=0.3),
            information,
            speed_kmh=20.0
        )

        self.assertEqual(result.throttle, 0.0)
        self.assertEqual(result.brake, 0.75)
        self.assertEqual(result.steer, 0.1)

    def test_red_light_holds_stopped_vehicle(self):
        vehicle = FakeVehicle(
            FakeTrafficLight("Red", distance=2.0)
        )
        information = self.safety.inspect(vehicle)

        result = self.safety.apply(
            FakeControl(),
            information,
            speed_kmh=0.5
        )

        self.assertEqual(result.brake, 1.0)

    def test_red_hold_stays_latched_if_carla_loses_the_light(self):
        traffic_light = FakeTrafficLight("Red", distance=2.0)
        vehicle = FakeVehicle(traffic_light)
        information = self.safety.inspect(vehicle)

        first_result = self.safety.apply(
            FakeControl(throttle=0.2),
            information,
            speed_kmh=0.5
        )
        vehicle.traffic_light = None
        second_information = self.safety.inspect(vehicle)
        second_result = self.safety.apply(
            FakeControl(throttle=0.2),
            second_information,
            speed_kmh=0.2
        )

        self.assertEqual(first_result.brake, 1.0)
        self.assertEqual(second_result.throttle, 0.0)
        self.assertEqual(second_result.brake, 1.0)
        self.assertEqual(
            second_information["safety_state"],
            self.safety.RED_BRAKING
        )

    def test_green_light_releases_latched_red_hold(self):
        traffic_light = FakeTrafficLight("Red", distance=2.0)
        vehicle = FakeVehicle(traffic_light)
        information = self.safety.inspect(vehicle)
        self.safety.apply(FakeControl(), information, speed_kmh=0.5)

        traffic_light.set_state("Green")
        vehicle.traffic_light = None
        green_information = self.safety.inspect(vehicle)
        requested = FakeControl(throttle=0.2)
        result = self.safety.apply(
            requested,
            green_information,
            speed_kmh=0.0
        )

        self.assertIs(result, requested)
        self.assertFalse(self.safety.red_hold_latched)

    def test_distant_red_light_does_not_brake_early(self):
        vehicle = FakeVehicle(
            FakeTrafficLight("Red", distance=15.0)
        )
        information = self.safety.inspect(vehicle)
        requested = FakeControl(throttle=0.3)

        result = self.safety.apply(
            requested,
            information,
            speed_kmh=20.0
        )

        self.assertIs(result, requested)
        self.assertEqual(
            information["safety_state"],
            self.safety.CLEAR
        )

    def test_stopped_vehicle_creeps_toward_stop_line(self):
        vehicle = FakeVehicle(
            FakeTrafficLight("Red", distance=5.0)
        )
        information = self.safety.inspect(vehicle)

        result = self.safety.apply(
            FakeControl(throttle=0.2),
            information,
            speed_kmh=0.5
        )

        self.assertEqual(result.brake, 0.0)
        self.assertEqual(result.throttle, 0.05)
        self.assertEqual(
            information["safety_state"],
            self.safety.RED_CREEPING
        )

    def test_distance_and_event_key_are_reported(self):
        information = self.safety.inspect(
            FakeVehicle(FakeTrafficLight("Red", actor_id=7))
        )

        self.assertEqual(information["distance_m"], 10.0)
        self.assertEqual(information["event_key"], (7, "RED"))


if __name__ == "__main__":
    unittest.main()
