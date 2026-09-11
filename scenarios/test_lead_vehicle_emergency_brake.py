import unittest

from scenarios.lead_vehicle_emergency_brake import (
    LeadVehicleEmergencyBrakeScenario
)
from scenarios.test_obstacle_ahead import FakeVehicle, FakeWorld


class LeadVehicleEmergencyBrakeScenarioTests(unittest.TestCase):
    def test_lead_vehicle_drives_then_brakes_after_delay(self):
        world = FakeWorld()
        scenario = LeadVehicleEmergencyBrakeScenario()

        actors = scenario.setup(world, FakeVehicle())
        lead_vehicle = actors[0]

        self.assertTrue(lead_vehicle.autopilot)
        self.assertEqual(lead_vehicle.transform.location.x, 16.0)
        scenario.update(3.9)
        self.assertEqual(lead_vehicle.control.brake, 0.0)

        scenario.update(4.0)

        self.assertFalse(lead_vehicle.autopilot)
        self.assertEqual(lead_vehicle.control.throttle, 0.0)
        self.assertEqual(lead_vehicle.control.brake, 1.0)
        self.assertTrue(scenario.emergency_braking_started)

    def test_emergency_braking_is_triggered_only_once(self):
        world = FakeWorld()
        scenario = LeadVehicleEmergencyBrakeScenario()
        lead_vehicle = scenario.setup(world, FakeVehicle())[0]

        scenario.update(4.0)
        first_control = lead_vehicle.control
        scenario.update(10.0)

        self.assertIs(lead_vehicle.control, first_control)


if __name__ == "__main__":
    unittest.main()
