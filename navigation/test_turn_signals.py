import unittest

from navigation.turn_signals import (
    HAZARD,
    LEFT,
    OFF,
    RIGHT,
    select_turn_signal
)


class TurnSignalTests(unittest.TestCase):
    def test_left_signal_is_used_while_approaching_turn(self):
        self.assertEqual(
            select_turn_signal({
                "mode": "APPROACH",
                "maneuver": "LEFT"
            }),
            LEFT
        )

    def test_right_signal_stays_on_inside_intersection(self):
        self.assertEqual(
            select_turn_signal({
                "mode": "INTERSECTION",
                "maneuver": "RIGHT"
            }),
            RIGHT
        )

    def test_signal_is_off_during_normal_lane_following(self):
        self.assertEqual(
            select_turn_signal({
                "mode": "AI",
                "maneuver": "LEFT"
            }),
            OFF
        )

    def test_straight_maneuver_does_not_signal(self):
        self.assertEqual(
            select_turn_signal({
                "mode": "APPROACH",
                "maneuver": "STRAIGHT"
            }),
            OFF
        )

    def test_hazards_override_turn_signal(self):
        self.assertEqual(
            select_turn_signal({
                "mode": "APPROACH",
                "maneuver": "RIGHT"
            }, hazards_active=True),
            HAZARD
        )


if __name__ == "__main__":
    unittest.main()
