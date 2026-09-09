"""Generate a short, clearly simulated 911 dialing sound."""

import math
import struct
import wave
from pathlib import Path


SAMPLE_RATE = 44100
AMPLITUDE = 0.22
OUTPUT_PATH = Path(__file__).with_name("simulated_call.wav")

DTMF_FREQUENCIES = {
    "1": (697.0, 1209.0),
    "9": (852.0, 1477.0)
}


def tone(frequencies, duration_seconds):
    frame_count = int(SAMPLE_RATE * duration_seconds)

    for frame in range(frame_count):
        time_seconds = frame / SAMPLE_RATE
        sample = sum(
            math.sin(2.0 * math.pi * frequency * time_seconds)
            for frequency in frequencies
        ) / len(frequencies)
        fade_frames = int(SAMPLE_RATE * 0.01)
        fade = min(
            1.0,
            frame / max(fade_frames, 1),
            (frame_count - frame - 1) / max(fade_frames, 1)
        )
        value = int(32767 * AMPLITUDE * sample * max(fade, 0.0))
        yield struct.pack("<hh", value, value)


def silence(duration_seconds):
    silent_frame = struct.pack("<hh", 0, 0)

    for _ in range(int(SAMPLE_RATE * duration_seconds)):
        yield silent_frame


def generate():
    with wave.open(str(OUTPUT_PATH), "wb") as output:
        output.setnchannels(2)
        output.setsampwidth(2)
        output.setframerate(SAMPLE_RATE)

        for digit in "911":
            output.writeframes(
                b"".join(tone(DTMF_FREQUENCIES[digit], 0.18))
            )
            output.writeframes(b"".join(silence(0.10)))

        # A short US-style ringback fragment signals that the fake call
        # attempt has moved from dialing to connecting.
        output.writeframes(b"".join(tone((440.0, 480.0), 1.0)))


if __name__ == "__main__":
    generate()
