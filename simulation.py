import time

import carla
import cv2

import sensors
from config import RUN_DURATION_SECONDS


def update_spectator(world, ego_vehicle):
    spectator = world.get_spectator()
    vehicle_transform = ego_vehicle.get_transform()

    spectator_location = vehicle_transform.transform(
        carla.Location(
            x=-8,
            z=4
        )
    )

    spectator_transform = carla.Transform(
        spectator_location,
        carla.Rotation(
            pitch=-15,
            yaw=vehicle_transform.rotation.yaw,
            roll=0
        )
    )

    spectator.set_transform(
        spectator_transform
    )


def draw_controller_information(frame, information):
    display_frame = frame.copy()

    if information is None:
        return display_frame

    lines = [
        (
            f"Raw steering: "
            f"{information['raw_steering']:+.4f}"
        ),
        (
            f"Applied steering: "
            f"{information['applied_steering']:+.4f}"
        ),
        (
            f"Throttle: "
            f"{information['throttle']:.2f}"
        ),
        (
            f"Brake: "
            f"{information['brake']:.2f}"
        )
    ]

    speed_kmh = information.get("speed_kmh")

    if speed_kmh is not None:
        lines.append(
            f"Speed: {speed_kmh:.1f} km/h"
        )

    overlay = display_frame.copy()

    cv2.rectangle(
        overlay,
        (10, 10),
        (400, 170),
        (0, 0, 0),
        thickness=-1
    )

    cv2.addWeighted(
        overlay,
        0.65,
        display_frame,
        0.35,
        0,
        display_frame
    )

    y_position = 38

    for line in lines:
        cv2.putText(
            display_frame,
            line,
            (25, y_position),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        y_position += 28

    return display_frame


def run_simulation(
    world,
    ego_vehicle,
    controller,
    data_collector=None
):
    controller.activate(ego_vehicle)

    end_time = time.time() + RUN_DURATION_SECONDS
    last_processed_frame_number = None
    controller_information = None

    try:
        while time.time() < end_time:
            world.wait_for_tick()

            update_spectator(
                world,
                ego_vehicle
            )

            frame, frame_number = (
                sensors.get_latest_frame()
            )

            is_new_frame = (
                frame is not None
                and frame_number is not None
                and frame_number
                != last_processed_frame_number
            )

            if is_new_frame:
                controller_information = (
                    controller.update(
                        vehicle=ego_vehicle,
                        frame=frame
                    )
                )

                if data_collector is not None:
                    data_collector.save_sample(
                        image=frame,
                        carla_frame=frame_number,
                        ego_vehicle=ego_vehicle
                    )

                last_processed_frame_number = (
                    frame_number
                )

            if frame is not None:
                display_frame = (
                    draw_controller_information(
                        frame,
                        controller_information
                    )
                )

                cv2.imshow(
                    "CARLA RGB Camera",
                    display_frame
                )

            pressed_key = cv2.waitKey(1) & 0xFF

            if pressed_key in (
                ord("q"),
                27
            ):
                break

    finally:
        controller.deactivate(ego_vehicle)