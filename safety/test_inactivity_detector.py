import unittest

from safety.inactivity_detector import (
    ControllerInactivityDetector
)


class ControllerInactivityDetectorTests(unittest.TestCase):
    def setUp(self):
        self.detector = ControllerInactivityDetector(
            warning_seconds=2.0,
            safe_stop_seconds=5.0,
            minimum_speed_kmh=3.0
        )

    def test_fresh_response_keeps_normal_state(self):
        state, inactive_seconds = self.detector.update(
            controller_responded=True,
            speed_kmh=20.0,
            now=10.0
        )

        self.assertEqual(
            state,
            ControllerInactivityDetector.NORMAL
        )
        self.assertEqual(inactive_seconds, 0.0)

    def test_missing_response_progresses_to_safe_stop(self):
        self.detector.update(True, 20.0, now=10.0)

        state, _ = self.detector.update(
            False,
            20.0,
            now=12.0
        )
        self.assertEqual(
            state,
            ControllerInactivityDetector.WARNING
        )

        state, inactive_seconds = self.detector.update(
            False,
            20.0,
            now=15.0
        )
        self.assertEqual(
            state,
            ControllerInactivityDetector.SAFE_STOP
        )
        self.assertEqual(inactive_seconds, 5.0)

    def test_stopped_vehicle_does_not_trigger_warning(self):
        self.detector.update(True, 20.0, now=10.0)

        state, inactive_seconds = self.detector.update(
            False,
            0.0,
            now=30.0
        )

        self.assertEqual(
            state,
            ControllerInactivityDetector.NORMAL
        )
        self.assertEqual(inactive_seconds, 0.0)

    def test_response_recovers_from_warning(self):
        self.detector.update(True, 20.0, now=10.0)
        self.detector.update(False, 20.0, now=13.0)

        state, inactive_seconds = self.detector.update(
            True,
            20.0,
            now=13.1
        )

        self.assertEqual(
            state,
            ControllerInactivityDetector.NORMAL
        )
        self.assertEqual(inactive_seconds, 0.0)

    def test_safe_stop_stays_latched_when_vehicle_stops(self):
        self.detector.update(True, 20.0, now=10.0)
        self.detector.update(False, 20.0, now=15.0)

        state, inactive_seconds = self.detector.update(
            False,
            0.0,
            now=16.0
        )

        self.assertEqual(
            state,
            ControllerInactivityDetector.SAFE_STOP
        )
        self.assertEqual(inactive_seconds, 6.0)

    def test_fresh_response_releases_safe_stop(self):
        self.detector.update(True, 20.0, now=10.0)
        self.detector.update(False, 20.0, now=15.0)

        state, inactive_seconds = self.detector.update(
            True,
            0.0,
            now=16.0
        )

        self.assertEqual(
            state,
            ControllerInactivityDetector.NORMAL
        )
        self.assertEqual(inactive_seconds, 0.0)


if __name__ == "__main__":
    unittest.main()
