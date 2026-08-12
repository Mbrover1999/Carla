

import csv
import time
from pathlib import Path

from config import PROJECT_ROOT


class SafetyLogger:
    def __init__(self):
        self.log_directory = (
            Path(PROJECT_ROOT)
            / "safety_logs"
        )

        self.log_path = (
            self.log_directory
            / "safety_log.csv"
        )

        self.csv_file = None
        self.csv_writer = None
        self.start_time = None
        self.last_safety_state = "CLEAR"
        self.collision_logged = False

    def start(self):
        self.log_directory.mkdir(
            parents=True,
            exist_ok=True
        )

        file_already_exists = (
            self.log_path.exists()
            and self.log_path.stat().st_size > 0
        )

        self.csv_file = open(
            self.log_path,
            "a",
            newline="",
            encoding="utf-8"
        )

        self.csv_writer = csv.writer(
            self.csv_file
        )

        if not file_already_exists:
            self.csv_writer.writerow([
                "timestamp_seconds",
                "speed_kmh",
                "obstacle_distance",
                "safety_state",
                "throttle",
                "brake",
                "collision",
                "collision_actor"
            ])

            self.csv_file.flush()

        self.start_time = time.time()

        print(
            f"Safety logger started: {self.log_path}"
        )

    def log_event(
        self,
        speed_kmh,
        obstacle_distance,
        safety_state,
        control,
        collision=False,
        collision_actor=None
    ):
        if self.csv_writer is None:
            raise RuntimeError(
                "Safety logger was not started"
            )

        safety_changed = (
            safety_state != self.last_safety_state
        )

        new_collision = (
            collision
            and not self.collision_logged
        )

        should_log = (
            safety_state != "CLEAR"
            and safety_changed
        ) or new_collision

        if not should_log:
            self.last_safety_state = safety_state
            return False

        timestamp_seconds = (
            time.time() - self.start_time
        )

        actor_type = ""

        if collision_actor is not None:
            actor_type = getattr(
                collision_actor,
                "type_id",
                "unknown"
            )

        self.csv_writer.writerow([
            f"{timestamp_seconds:.3f}",
            f"{speed_kmh:.3f}",
            "" if obstacle_distance is None
            else f"{obstacle_distance:.3f}",
            safety_state,
            f"{control.throttle:.3f}",
            f"{control.brake:.3f}",
            collision,
            actor_type
        ])

        self.csv_file.flush()

        self.last_safety_state = safety_state

        if new_collision:
            self.collision_logged = True

        print(
            "Safety event:",
            safety_state,
            f"speed={speed_kmh:.1f} km/h",
            f"distance={obstacle_distance}",
            f"collision={collision}"
        )

        return True

    def close(self):
        if self.csv_file is None:
            return

        self.csv_file.flush()
        self.csv_file.close()

        self.csv_file = None
        self.csv_writer = None

        print("Safety logger closed")