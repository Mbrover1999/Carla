import csv
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from safety.safety_logger import SafetyLogger


class SafetyLoggerTests(unittest.TestCase):
    def test_custom_run_log_contains_only_current_run(self):
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "free_drive.csv"
            logger = SafetyLogger(log_path=log_path)
            logger.start()
            logger.log_event(
                speed_kmh=20.0,
                obstacle_distance=6.0,
                safety_state="SLOWING",
                control=SimpleNamespace(throttle=0.1, brake=0.0),
                event_key="event-1"
            )
            logger.close()

            with log_path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.reader(stream))

        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0][3], "safety_state")
        self.assertEqual(rows[1][3], "SLOWING")


if __name__ == "__main__":
    unittest.main()
