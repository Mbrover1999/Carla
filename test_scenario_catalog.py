import unittest

from scenario_catalog import (
    MAX_DURATION_MINUTES,
    MAX_TRAFFIC_VEHICLES,
    SCENARIOS,
    SimulationSettings,
    get_scenario
)


class ScenarioCatalogTests(unittest.TestCase):
    def test_catalog_contains_expected_scenarios(self):
        self.assertEqual(
            [scenario.scenario_id for scenario in SCENARIOS],
            [
                "free_drive",
                "obstacle_ahead",
                "cross_traffic",
                "lane_departure",
                "red_traffic_light",
                "driver_inactivity"
            ]
        )

    def test_free_drive_settings_are_valid(self):
        settings = SimulationSettings(
            scenario_id="free_drive",
            duration_minutes=5,
            traffic_vehicles=20
        ).validate()

        self.assertEqual(settings.duration_seconds, 300)

    def test_duration_must_be_in_range(self):
        with self.assertRaises(ValueError):
            SimulationSettings(
                scenario_id="free_drive",
                duration_minutes=MAX_DURATION_MINUTES + 1,
                traffic_vehicles=20
            ).validate()

    def test_traffic_count_must_be_in_range(self):
        with self.assertRaises(ValueError):
            SimulationSettings(
                scenario_id="free_drive",
                duration_minutes=5,
                traffic_vehicles=MAX_TRAFFIC_VEHICLES + 1
            ).validate()

    def test_unimplemented_scenario_cannot_start(self):
        with self.assertRaisesRegex(ValueError, "not implemented"):
            SimulationSettings(
                scenario_id="cross_traffic",
                duration_minutes=5,
                traffic_vehicles=20
            ).validate()

    def test_unknown_scenario_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Unknown scenario"):
            get_scenario("unknown")


if __name__ == "__main__":
    unittest.main()
