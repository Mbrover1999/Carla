import platform
import shutil
import subprocess

from config import SAFETY_SOUND_ENABLED


class SafetyAlertManager:
    def __init__(self, sound_enabled=SAFETY_SOUND_ENABLED):
        self.sound_enabled = sound_enabled
        self.last_reason = None
        self.sound_process = None

    def update(self, reason, urgent=False):
        is_new_alert = (
            reason is not None
            and reason != self.last_reason
        )

        self.last_reason = reason

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

        if system_name == "Darwin":
            self._play_macos_sound(urgent)
            return

        if system_name == "Windows":
            self._play_windows_sound(urgent)
            return

        # Terminal bell is a dependency-free fallback on Linux and
        # other platforms. Whether it is audible depends on terminal
        # sound settings.
        print("\a", end="", flush=True)

    def _play_macos_sound(self, urgent):
        afplay_path = shutil.which("afplay")

        if afplay_path is None:
            print("\a", end="", flush=True)
            return

        sound_name = (
            "Sosumi.aiff"
            if urgent
            else "Glass.aiff"
        )

        sound_path = (
            "/System/Library/Sounds/"
            f"{sound_name}"
        )

        self.sound_process = subprocess.Popen(
            [afplay_path, sound_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    @staticmethod
    def _play_windows_sound(urgent):
        try:
            import winsound

            sound_type = (
                winsound.MB_ICONHAND
                if urgent
                else winsound.MB_ICONEXCLAMATION
            )

            winsound.MessageBeep(sound_type)
        except (ImportError, RuntimeError):
            print("\a", end="", flush=True)
