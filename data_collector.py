import csv
import math
from pathlib import Path

import cv2

from config import (
    DATASET_DIRECTORY,
    IMAGES_DIRECTORY,
    DRIVING_LOG_FILENAME,
    SAVE_EVERY_N_FRAMES,
    IMAGE_FILE_EXTENSION,
    JPEG_QUALITY
)


class DataCollector:
    def __init__(self):
        self.dataset_path = Path(DATASET_DIRECTORY)
        self.images_path = (
            self.dataset_path / IMAGES_DIRECTORY
        )

        self.csv_path = (
            self.dataset_path / DRIVING_LOG_FILENAME
        )

        self.csv_file = None
        self.csv_writer = None
        self.saved_samples = 0
        self.last_saved_frame = None

    def start(self):
        self.images_path.mkdir(
            parents=True,
            exist_ok=True
        )

        file_already_exists = self.csv_path.exists()
        file_has_content = (
            file_already_exists
            and self.csv_path.stat().st_size > 0
        )

        self.csv_file = self.csv_path.open(
            mode="a",
            newline="",
            encoding="utf-8"
        )

        self.csv_writer = csv.writer(
            self.csv_file
        )

        if not file_has_content:
            self.csv_writer.writerow([
                "image_path",
                "frame",
                "steering",
                "throttle",
                "brake",
                "speed"
            ])

            self.csv_file.flush()

        print(
            f"Data collector started: "
            f"{self.dataset_path.resolve()}"
        )

    def save_sample(
        self,
        image,
        carla_frame,
        ego_vehicle
    ):
        if self.csv_writer is None:
            raise RuntimeError(
                "Data collector was not started"
            )

        if image is None:
            return False

        # Do not save the same camera frame twice.
        if carla_frame == self.last_saved_frame:
            return False

        self.last_saved_frame = carla_frame

        # Save only one out of every N camera frames.
        control = ego_vehicle.get_control()
        speed = self._calculate_speed(ego_vehicle)

        # Vehicle is almost standing still.
        if speed < 0.5:
            save_interval = 20

        # Sharp turn.
        elif abs(control.steer) > 0.15:
            save_interval = 2

        # Medium turn.
        elif abs(control.steer) > 0.05:
            save_interval = 3

        # Straight driving.
        else:
            save_interval = 6

        if carla_frame % save_interval != 0:
            return False

        image_filename = (
            f"frame_{carla_frame:08d}"
            f"{IMAGE_FILE_EXTENSION}"
        )

        full_image_path = (
            self.images_path / image_filename
        )

        image_saved = cv2.imwrite(
            str(full_image_path),
            image,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                JPEG_QUALITY
            ]
        )

        if not image_saved:
            print(
                f"Failed to save image: "
                f"{full_image_path}"
            )

            return False

        relative_image_path = (
            Path(IMAGES_DIRECTORY)
            / image_filename
        )

        self.csv_writer.writerow([
            relative_image_path.as_posix(),
            carla_frame,
            float(control.steer),
            float(control.throttle),
            float(control.brake),
            speed
        ])

        self.saved_samples += 1

        if self.saved_samples % 50 == 0:
            self.csv_file.flush()

            print(
                f"Saved {self.saved_samples} samples"
            )

        return True

    def close(self):
        if self.csv_file is None:
            return

        self.csv_file.flush()
        self.csv_file.close()

        self.csv_file = None
        self.csv_writer = None

        print(
            f"Data collector closed. "
            f"Saved {self.saved_samples} samples."
        )

    @staticmethod
    def _calculate_speed(vehicle):
        velocity = vehicle.get_velocity()

        return math.sqrt(
            velocity.x ** 2
            + velocity.y ** 2
            + velocity.z ** 2
        )