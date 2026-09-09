import platform
import shutil
import subprocess
from pathlib import Path

from config import (
    SAFETY_SOUND_ENABLED,
    SAFETY_URGENT_SOUND_PATH,
    SAFETY_WARNING_SOUND_PATH
)


class SafetyAlertManager:
    def __init__(
        self,
        sound_enabled=SAFETY_SOUND_ENABLED,
        warning_sound_path=SAFETY_WARNING_SOUND_PATH,
        urgent_sound_path=SAFETY_URGENT_SOUND_PATH
    ):
        self.sound_enabled = sound_enabled
        self.warning_sound_path = Path(warning_sound_path)
        self.urgent_sound_path = Path(urgent_sound_path)
        self.last_reason = None
        self.last_alert_identity = None
        self.sound_process = None

    def update(
        self,
        reason,
        urgent=False,
        event_key=None
    ):
        alert_identity = (
            None
            if reason is None
            else (reason, event_key)
        )

        is_new_alert = (
            reason is not None
            and alert_identity != self.last_alert_identity
        )

        self.last_reason = reason
        self.last_alert_identity = alert_identity

        if is_new_alert and self.sound_enabled:
            self._play_sound(urgent=urgent)

        return is_new_alert

    def close(self):
        if (
            self.sound_process is not None
            and self.sound_process.poll() is None
        ):
            try:
                self.sound_process.terminate()
            except OSError:
                pass

        self.sound_process = None

    def _play_sound(self, urgent):
        system_name = platform.system()
        sound_path = self._sound_path(urgent)

        if system_name == "Darwin":
            self._play_macos_sound(sound_path, urgent)
            return

        if system_name == "Windows":
            self._play_windows_sound(sound_path, urgent)
            return

        if sound_path is not None:
            for player_name in ("paplay", "aplay"):
                player_path = shutil.which(player_name)

                if player_path is not None:
                    self.sound_process = subprocess.Popen(
                        [player_path, str(sound_path)],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
                    return

        # Terminal bell is a dependency-free fallback on Linux and
        # other platforms. Whether it is audible depends on terminal
        # sound settings.
        print("\a", end="", flush=True)

    def _play_macos_sound(self, sound_path, urgent):
        afplay_path = shutil.which("afplay")

        if afplay_path is None:
            print("\a", end="", flush=True)
            return

        if sound_path is None:
            sound_name = (
                "Sosumi.aiff"
                if urgent
                else "Glass.aiff"
            )
            sound_path = Path(
                "/System/Library/Sounds"
            ) / sound_name

        self.sound_process = subprocess.Popen(
            [afplay_path, str(sound_path)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    @staticmethod
    def _play_windows_sound(sound_path, urgent):
        try:
            import winsound

            if sound_path is not None:
                winsound.PlaySound(
                    str(sound_path),
                    winsound.SND_FILENAME
                    | winsound.SND_ASYNC
                    | winsound.SND_NODEFAULT
                )
            else:
                sound_type = (
                    winsound.MB_ICONHAND
                    if urgent
                    else winsound.MB_ICONEXCLAMATION
                )
                winsound.MessageBeep(sound_type)
        except (ImportError, RuntimeError):
            print("\a", end="", flush=True)

    def _sound_path(self, urgent):
        sound_path = (
            self.urgent_sound_path
            if urgent
            else self.warning_sound_path
        )

        return sound_path if sound_path.is_file() else None
