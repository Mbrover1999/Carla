import unittest

from scenario_catalog import (
    MAX_DURATION_MINUTES,
    SCENARIOS,
    TRAFFIC_PRESETS,
    SimulationSettings,
    get_scenario,
    get_traffic_preset_by_count,
    get_traffic_preset_by_display_name
)


class ScenarioCatalogTests(unittest.TestCase):
    def test_catalog_contains_expected_scenarios(self):
        self.assertEqual(
            [scenario.scenario_id for scenario in SCENARIOS],
            [
                "free_drive",
                "obstacle_ahead",
                "lead_vehicle_emergency_brake",
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

    def test_obstacle_ahead_is_available(self):
        settings = SimulationSettings(
            scenario_id="obstacle_ahead",
            duration_minutes=5,
            traffic_vehicles=20
        ).validate()

        self.assertEqual(settings.scenario_id, "obstacle_ahead")

    def test_lead_vehicle_emergency_brake_is_available(self):
        settings = SimulationSettings(
            scenario_id="lead_vehicle_emergency_brake",
            duration_minutes=1,
            traffic_vehicles=0
        ).validate()

        self.assertEqual(
            settings.scenario_id,
            "lead_vehicle_emergency_brake"
        )

    def test_duration_must_be_in_range(self):
        with self.assertRaises(ValueError):
            SimulationSettings(
                scenario_id="free_drive",
                duration_minutes=MAX_DURATION_MINUTES + 1,
                traffic_vehicles=20
            ).validate()

    def test_traffic_count_must_match_a_preset(self):
        with self.assertRaises(ValueError):
            SimulationSettings(
                scenario_id="free_drive",
                duration_minutes=5,
                traffic_vehicles=25
            ).validate()

    def test_traffic_presets_have_expected_counts(self):
        self.assertEqual(
            [preset.vehicle_count for preset in TRAFFIC_PRESETS],
            [0, 10, 20, 30, 40]
        )

    def test_traffic_preset_can_be_resolved_for_interface(self):
        preset = get_traffic_preset_by_count(40)

        self.assertEqual(preset.title, "Heavy Traffic")
        self.assertEqual(
            get_traffic_preset_by_display_name(
                preset.display_name
            ),
            preset
        )

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
