import unittest
from types import SimpleNamespace

from journey_evaluator import JourneyEvaluator


def location(x, y=0.0):
    return SimpleNamespace(x=x, y=y)


class JourneyEvaluatorTests(unittest.TestCase):
    def test_safe_efficient_completed_drive_scores_high(self):
        evaluator = JourneyEvaluator()

        for index in range(20):
            evaluator.update(
                location(index * 2.0),
                speed_kmh=29.0,
                target_speed_kmh=30.0,
                safety_state="CLEAR"
            )

        result = evaluator.result(60.0, "COMPLETED")

        self.assertGreaterEqual(result["score"], 95)
        self.assertEqual(result["collisions"], 0)
        self.assertAlmostEqual(result["distance_km"], 0.038)

    def test_collision_and_lane_departure_reduce_score(self):
        evaluator = JourneyEvaluator()
        evaluator.update(
            location(0.0),
            20.0,
            30.0,
            "LANE_INVASION",
            collision=True,
            lane_event_key=101
        )
        evaluator.update(
            location(1.0),
            0.0,
            30.0,
            "LANE_INVASION",
            collision=True,
            lane_event_key=101
        )

        result = evaluator.result(10.0, "COMPLETED")

        self.assertEqual(result["collisions"], 1)
        self.assertEqual(result["lane_departures"], 1)
        self.assertEqual(result["rating"], "Unsafe")

    def test_normal_safety_intervention_is_counted_without_safety_penalty(self):
        evaluator = JourneyEvaluator()
        evaluator.update(location(0.0), 20.0, 30.0, "CLEAR")
        evaluator.update(location(1.0), 15.0, 30.0, "SLOWING")

        result = evaluator.result(10.0, "COMPLETED")

        self.assertEqual(result["safety_interventions"], 1)
        self.assertEqual(result["emergency_interventions"], 0)


if __name__ == "__main__":
    unittest.main()
