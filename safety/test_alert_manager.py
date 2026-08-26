import unittest
from unittest.mock import patch

from safety.alert_manager import SafetyAlertManager


class SafetyAlertManagerTests(unittest.TestCase):
    def test_sound_plays_only_when_alert_changes(self):
        manager = SafetyAlertManager(sound_enabled=True)

        with patch.object(
            manager,
            "_play_sound"
        ) as play_sound:
            self.assertTrue(
                manager.update("Obstacle ahead", urgent=False)
            )
            self.assertFalse(
                manager.update("Obstacle ahead", urgent=False)
            )
            self.assertTrue(
                manager.update("Emergency braking", urgent=True)
            )

        self.assertEqual(play_sound.call_count, 2)

    def test_alert_can_trigger_again_after_clear_state(self):
        manager = SafetyAlertManager(sound_enabled=True)

        with patch.object(
            manager,
            "_play_sound"
        ) as play_sound:
            manager.update("Obstacle ahead")
            manager.update(None)
            manager.update("Obstacle ahead")

        self.assertEqual(play_sound.call_count, 2)

    def test_disabled_sound_still_reports_new_alert(self):
        manager = SafetyAlertManager(sound_enabled=False)

        with patch.object(
            manager,
            "_play_sound"
        ) as play_sound:
            is_new_alert = manager.update("Safe stop")

        self.assertTrue(is_new_alert)
        play_sound.assert_not_called()


if __name__ == "__main__":
    unittest.main()
