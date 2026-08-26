import math
import time

import carla
import cv2

import sensors

from config import (
    CONTROLLER_INACTIVITY_ENABLED,
    CONTROLLER_INACTIVITY_SAFE_STOP_BRAKE,
    RUN_DURATION_SECONDS,
    SAFETY_ENABLED
)
from safety.inactivity_detector import (
    ControllerInactivityDetector
)
from safety.alert_manager import SafetyAlertManager
from safety.safety_layer import SafetyLayer
from safety.safety_logger import SafetyLogger


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


def draw_controller_information(
    frame,
    information,
    obstacle_safety_state="DISABLED",
    inactivity_state="DISABLED",
    inactivity_seconds=0.0,
    inactivity_test_active=False,
    intervention_reason=None,
    intervention_urgent=False,
    obstacle_distance=None,
    collision_detected=False
):
    display_frame = frame.copy()
    lines = []

    if information is not None:
        lines.extend([
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
        ])

        speed_kmh = information.get("speed_kmh")

        if speed_kmh is not None:
            lines.append(
                f"Speed: {speed_kmh:.1f} km/h"
            )

    lines.append(
        f"Obstacle safety: {obstacle_safety_state}"
    )

    inactivity_text = inactivity_state

    if inactivity_state in (
        ControllerInactivityDetector.WARNING,
        ControllerInactivityDetector.SAFE_STOP
    ):
        inactivity_text += (
            f" ({inactivity_seconds:.1f}s)"
        )

    lines.append(
        f"Controller: {inactivity_text}"
    )

    lines.append(
        "Inactivity test: "
        f"{'ACTIVE' if inactivity_test_active else 'OFF'} "
        "[I]"
    )

    if obstacle_distance is not None:
        lines.append(
            f"Obstacle: {obstacle_distance:.1f} m"
        )

    lines.append(
        f"Collision: {'YES' if collision_detected else 'NO'}"
    )

    overlay = display_frame.copy()

    cv2.rectangle(
        overlay,
        (10, 10),
        (520, 320),
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

    if intervention_reason is not None:
        reason_lines = intervention_reason.split(" | ")
        banner_color = (
            (0, 0, 210)
            if intervention_urgent
            else (0, 140, 255)
        )

        banner_height = 24 + 38 * len(reason_lines)
        banner_top = (
            display_frame.shape[0] - banner_height
        )

        cv2.rectangle(
            display_frame,
            (0, banner_top),
            (display_frame.shape[1], display_frame.shape[0]),
            banner_color,
            thickness=-1
        )

        for line_number, reason_line in enumerate(
            reason_lines
        ):
            prefix = (
                "SAFETY: "
                if line_number == 0
                else "        "
            )

            cv2.putText(
                display_frame,
                f"{prefix}{reason_line}",
                (
                    20,
                    banner_top + 38 * (line_number + 1)
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.72,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )

    return display_frame


def calculate_speed_kmh(vehicle):
    velocity = vehicle.get_velocity()

    speed_mps = math.sqrt(
        velocity.x ** 2
        + velocity.y ** 2
        + velocity.z ** 2
    )

    return speed_mps * 3.6


def combine_safety_states(
    obstacle_safety_state,
    inactivity_state
):
    active_states = []

    if obstacle_safety_state not in (
        "CLEAR",
        "DISABLED"
    ):
        active_states.append(obstacle_safety_state)

    if inactivity_state not in (
        ControllerInactivityDetector.NORMAL,
        "DISABLED"
    ):
        active_states.append(inactivity_state)

    if active_states:
        return " + ".join(active_states)

    if (
        obstacle_safety_state == "DISABLED"
        and inactivity_state == "DISABLED"
    ):
        return "DISABLED"

    return "CLEAR"


def get_intervention_reason(
    obstacle_safety_state,
    inactivity_state
):
    reasons = []
    urgent = False

    obstacle_reasons = {
        "SLOWING": "Obstacle ahead - slowing down",
        "BRAKING": "Obstacle ahead - braking",
        "EMERGENCY": "Obstacle ahead - EMERGENCY BRAKING"
    }

    obstacle_reason = obstacle_reasons.get(
        obstacle_safety_state
    )

    if obstacle_reason is not None:
        reasons.append(obstacle_reason)

    if obstacle_safety_state == "EMERGENCY":
        urgent = True

    inactivity_reasons = {
        ControllerInactivityDetector.WARNING: (
            "Controller inactivity warning"
        ),
        ControllerInactivityDetector.SAFE_STOP: (
            "Controller inactive - SAFE STOP"
        )
    }

    inactivity_reason = inactivity_reasons.get(
        inactivity_state
    )

    if inactivity_reason is not None:
        reasons.append(inactivity_reason)

    if inactivity_state == (
        ControllerInactivityDetector.SAFE_STOP
    ):
        urgent = True

    if not reasons:
        return None, False

    return " | ".join(reasons), urgent


def run_simulation(
    world,
    ego_vehicle,
    controller,
    data_collector=None
):
    controller.activate(ego_vehicle)
    safety_layer = SafetyLayer()
    inactivity_detector = ControllerInactivityDetector()
    alert_manager = SafetyAlertManager()
    safety_logger = SafetyLogger()
    safety_logger.start()

    end_time = time.time() + RUN_DURATION_SECONDS
    last_processed_frame_number = None
    controller_information = None
    requested_control = None
    obstacle_safety_state = "DISABLED"
    inactivity_state = "DISABLED"
    inactivity_seconds = 0.0
    inactivity_test_active = False
    intervention_reason = None
    intervention_urgent = False
    controller_error_active = False
    obstacle_distance = None
    collision_detected = False
    collision_actor = None

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
                controller_responded = False

                if (
                    not inactivity_test_active
                    or requested_control is None
                ):
                    try:
                        (
                            new_control,
                            new_information
                        ) = controller.update(
                            vehicle=ego_vehicle,
                            frame=frame
                        )

                        requested_control = new_control
                        controller_information = (
                            new_information
                        )
                        controller_responded = True

                        if controller_error_active:
                            print(
                                "Controller response recovered."
                            )

                        controller_error_active = False
                    except Exception as error:
                        if not controller_error_active:
                            print(
                                "Controller stopped responding:",
                                repr(error)
                            )

                        controller_error_active = True

                        # With no previous valid command, fail safe
                        # immediately. Otherwise keep the last command
                        # while the inactivity watchdog counts down.
                        if requested_control is None:
                            requested_control = (
                                carla.VehicleControl(
                                    steer=0.0,
                                    throttle=0.0,
                                    brake=1.0
                                )
                            )

                speed_kmh = calculate_speed_kmh(
                    ego_vehicle
                )

                (
                    obstacle_distance,
                    _
                ) = sensors.get_latest_obstacle()

                (
                    collision_detected,
                    collision_actor,
                    _
                ) = sensors.get_latest_collision()

                if SAFETY_ENABLED:
                    (
                        final_control,
                        obstacle_safety_state
                    ) = safety_layer.apply(
                        requested_control=requested_control,
                        obstacle_distance=obstacle_distance,
                        speed_kmh=speed_kmh
                    )
                else:
                    final_control = requested_control
                    obstacle_safety_state = "DISABLED"

                if (
                    SAFETY_ENABLED
                    and CONTROLLER_INACTIVITY_ENABLED
                ):
                    (
                        inactivity_state,
                        inactivity_seconds
                    ) = inactivity_detector.update(
                        controller_responded=(
                            controller_responded
                        ),
                        speed_kmh=speed_kmh
                    )
                else:
                    inactivity_state = "DISABLED"
                    inactivity_seconds = 0.0

                if inactivity_state == (
                    ControllerInactivityDetector.SAFE_STOP
                ):
                    final_control = carla.VehicleControl(
                        steer=0.0,
                        throttle=0.0,
                        brake=(
                            CONTROLLER_INACTIVITY_SAFE_STOP_BRAKE
                        )
                    )

                (
                    intervention_reason,
                    intervention_urgent
                ) = get_intervention_reason(
                    obstacle_safety_state,
                    inactivity_state
                )

                alert_manager.update(
                    reason=intervention_reason,
                    urgent=intervention_urgent
                )

                ego_vehicle.apply_control(
                    final_control
                )

                safety_logger.log_event(
                    speed_kmh=speed_kmh,
                    obstacle_distance=obstacle_distance,
                    safety_state=combine_safety_states(
                        obstacle_safety_state,
                        inactivity_state
                    ),
                    control=final_control,
                    collision=collision_detected,
                    collision_actor=collision_actor
                )

                if controller_information is not None:
                    controller_information[
                        "applied_steering"
                    ] = final_control.steer

                    controller_information[
                        "throttle"
                    ] = final_control.throttle

                    controller_information[
                        "brake"
                    ] = final_control.brake

                if (
                    data_collector is not None
                    and not inactivity_test_active
                ):
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
                        controller_information,
                        obstacle_safety_state,
                        inactivity_state,
                        inactivity_seconds,
                        inactivity_test_active,
                        intervention_reason,
                        intervention_urgent,
                        obstacle_distance,
                        collision_detected
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

            if pressed_key in (
                ord("i"),
                ord("I")
            ):
                inactivity_test_active = (
                    not inactivity_test_active
                )

                if inactivity_test_active:
                    print(
                        "Inactivity test enabled: controller "
                        "output is frozen. Press I to resume."
                    )
                else:
                    inactivity_detector.reset()
                    print(
                        "Inactivity test disabled: controller "
                        "resumed."
                    )

    finally:
        alert_manager.close()
        safety_logger.close()
        controller.deactivate(ego_vehicle)
