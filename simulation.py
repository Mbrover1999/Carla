import math
import time

import carla
import cv2

import sensors

from config import (
    CONTROLLER_INACTIVITY_ENABLED,
    CONTROLLER_INACTIVITY_SAFE_STOP_BRAKE,
    LANE_INVASION_ENABLED,
    LANE_KEEPING_ENABLED,
    RUN_DURATION_SECONDS,
    SAFETY_ENABLED,
    TRAFFIC_LIGHT_DETECTION_ENABLED
)
from safety.inactivity_detector import (
    ControllerInactivityDetector
)
from safety.alert_manager import SafetyAlertManager
from safety.lane_keeping import LaneKeepingAssist
from safety.safety_layer import SafetyLayer
from safety.safety_logger import SafetyLogger
from safety.traffic_light_safety import TrafficLightSafety


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
    lane_invasion_detected=False,
    lane_markings=None,
    lane_keeping_enabled=False,
    lane_keeping_information=None,
    traffic_light_information=None,
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

    lane_text = "NO"

    if lane_invasion_detected:
        marking_text = ", ".join(lane_markings or [])
        lane_text = (
            f"YES ({marking_text})"
            if marking_text
            else "YES"
        )

    lines.append(
        f"Lane departure: {lane_text}"
    )

    if traffic_light_information is None:
        traffic_light_state = TrafficLightSafety.NONE
        traffic_light_distance = None
    else:
        traffic_light_state = traffic_light_information[
            "light_state"
        ]
        traffic_light_distance = traffic_light_information.get(
            "distance_m"
        )

    traffic_light_text = traffic_light_state

    if traffic_light_distance is not None:
        traffic_light_text += (
            f" ({traffic_light_distance:.1f} m)"
        )

    lines.append(
        f"Traffic light: {traffic_light_text}"
    )

    if lane_keeping_information is None:
        lane_keeping_state = LaneKeepingAssist.DISABLED
    else:
        lane_keeping_state = lane_keeping_information["state"]

    correction_text = ""

    if (
        lane_keeping_information is not None
        and lane_keeping_state in (
            LaneKeepingAssist.CORRECTING_LEFT,
            LaneKeepingAssist.CORRECTING_RIGHT
        )
    ):
        correction_text = (
            " "
            f"({lane_keeping_information['steering_correction']:+.3f})"
        )

    lines.append(
        f"Lane keeping: {lane_keeping_state}{correction_text} "
        f"[L: {'ON' if lane_keeping_enabled else 'OFF'}]"
    )

    if lane_keeping_information is not None:
        lateral_offset = lane_keeping_information.get(
            "lateral_offset_m"
        )
        heading_error = lane_keeping_information.get(
            "heading_error_deg"
        )

        if (
            lateral_offset is not None
            and heading_error is not None
        ):
            lines.append(
                f"Lane offset: {lateral_offset:+.2f} m  "
                f"Heading: {heading_error:+.1f} deg"
            )

    overlay = display_frame.copy()

    cv2.rectangle(
        overlay,
        (10, 10),
        (
            560,
            min(
                display_frame.shape[0] - 10,
                20 + 28 * len(lines)
            )
        ),
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
    inactivity_state,
    lane_invasion_state="CLEAR",
    lane_keeping_state="CLEAR",
    traffic_light_state="CLEAR"
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

    if lane_invasion_state != "CLEAR":
        active_states.append(lane_invasion_state)

    if lane_keeping_state != "CLEAR":
        active_states.append(lane_keeping_state)

    if traffic_light_state != TrafficLightSafety.CLEAR:
        active_states.append(traffic_light_state)

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
    inactivity_state,
    lane_invasion_detected=False,
    lane_markings=None,
    lane_keeping_state=None,
    traffic_light_state=None
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

    if lane_invasion_detected:
        marking_text = ", ".join(lane_markings or [])
        lane_reason = "Lane departure detected"

        if marking_text:
            lane_reason += f" ({marking_text})"

        reasons.append(lane_reason)

    lane_keeping_reasons = {
        LaneKeepingAssist.CORRECTING_LEFT: (
            "Lane keeping assist - steering left"
        ),
        LaneKeepingAssist.CORRECTING_RIGHT: (
            "Lane keeping assist - steering right"
        )
    }

    lane_keeping_reason = lane_keeping_reasons.get(
        lane_keeping_state
    )

    if lane_keeping_reason is not None:
        reasons.append(lane_keeping_reason)

    traffic_light_reasons = {
        TrafficLightSafety.YELLOW_WARNING: (
            "Yellow traffic light ahead"
        ),
        TrafficLightSafety.RED_BRAKING: (
            "Red traffic light - stopping"
        )
    }

    traffic_light_reason = traffic_light_reasons.get(
        traffic_light_state
    )

    if traffic_light_reason is not None:
        reasons.append(traffic_light_reason)

    if traffic_light_state == TrafficLightSafety.RED_BRAKING:
        urgent = True

    if not reasons:
        return None, False

    return " | ".join(reasons), urgent


def combine_event_keys(*event_keys):
    active_keys = tuple(
        event_key
        for event_key in event_keys
        if event_key is not None
    )

    if not active_keys:
        return None

    return active_keys


def run_simulation(
    world,
    ego_vehicle,
    controller,
    data_collector=None
):
    controller.activate(ego_vehicle)
    safety_layer = SafetyLayer()
    inactivity_detector = ControllerInactivityDetector()
    lane_keeping_assist = LaneKeepingAssist(
        world.get_map()
    )
    traffic_light_safety = TrafficLightSafety()
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
    lane_invasion_detected = False
    lane_markings = []
    lane_event_key = None
    lane_keeping_enabled = LANE_KEEPING_ENABLED
    lane_keeping_information = LaneKeepingAssist.status(
        (
            LaneKeepingAssist.LOW_SPEED
            if lane_keeping_enabled
            else LaneKeepingAssist.DISABLED
        )
    )
    traffic_light_information = TrafficLightSafety.information()
    safety_event_key = None

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

                if LANE_INVASION_ENABLED:
                    (
                        lane_invasion_detected,
                        lane_markings,
                        lane_event_key
                    ) = sensors.get_latest_lane_invasion()
                else:
                    lane_invasion_detected = False
                    lane_markings = []
                    lane_event_key = None

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
                    and TRAFFIC_LIGHT_DETECTION_ENABLED
                ):
                    traffic_light_information = (
                        traffic_light_safety.inspect(
                            ego_vehicle
                        )
                    )

                    final_control = traffic_light_safety.apply(
                        requested_control=final_control,
                        traffic_light_information=(
                            traffic_light_information
                        ),
                        speed_kmh=speed_kmh
                    )
                else:
                    traffic_light_information = (
                        TrafficLightSafety.information()
                    )

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

                lane_keeping_allowed = (
                    SAFETY_ENABLED
                    and lane_keeping_enabled
                    and inactivity_state
                    != ControllerInactivityDetector.SAFE_STOP
                    and obstacle_safety_state not in (
                        "BRAKING",
                        "EMERGENCY"
                    )
                    and traffic_light_information[
                        "safety_state"
                    ] != TrafficLightSafety.RED_BRAKING
                )

                if lane_keeping_allowed:
                    (
                        final_control,
                        lane_keeping_information
                    ) = lane_keeping_assist.apply(
                        vehicle=ego_vehicle,
                        requested_control=final_control,
                        speed_kmh=speed_kmh
                    )
                else:
                    lane_keeping_state = (
                        LaneKeepingAssist.DISABLED
                        if (
                            not SAFETY_ENABLED
                            or not lane_keeping_enabled
                        )
                        else LaneKeepingAssist.SUPPRESSED
                    )

                    lane_keeping_information = (
                        LaneKeepingAssist.status(
                            lane_keeping_state
                        )
                    )

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
                    inactivity_state,
                    lane_invasion_detected,
                    lane_markings,
                    lane_keeping_information["state"],
                    traffic_light_information["safety_state"]
                )

                traffic_light_event_key = None

                if traffic_light_information[
                    "safety_state"
                ] != TrafficLightSafety.CLEAR:
                    traffic_light_event_key = (
                        traffic_light_information["event_key"]
                    )

                safety_event_key = combine_event_keys(
                    lane_event_key,
                    traffic_light_event_key
                )

                alert_manager.update(
                    reason=intervention_reason,
                    urgent=intervention_urgent,
                    event_key=safety_event_key
                )

                ego_vehicle.apply_control(
                    final_control
                )

                safety_logger.log_event(
                    speed_kmh=speed_kmh,
                    obstacle_distance=obstacle_distance,
                    safety_state=combine_safety_states(
                        obstacle_safety_state,
                        inactivity_state,
                        (
                            "LANE_INVASION"
                            if lane_invasion_detected
                            else "CLEAR"
                        ),
                        (
                            lane_keeping_information["state"]
                            if lane_keeping_information["state"] in (
                                LaneKeepingAssist.CORRECTING_LEFT,
                                LaneKeepingAssist.CORRECTING_RIGHT
                            )
                            else "CLEAR"
                        ),
                        traffic_light_information[
                            "safety_state"
                        ]
                    ),
                    control=final_control,
                    collision=collision_detected,
                    collision_actor=collision_actor,
                    event_key=safety_event_key
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
                        frame=frame,
                        information=controller_information,
                        obstacle_safety_state=(
                            obstacle_safety_state
                        ),
                        inactivity_state=inactivity_state,
                        inactivity_seconds=inactivity_seconds,
                        inactivity_test_active=(
                            inactivity_test_active
                        ),
                        intervention_reason=intervention_reason,
                        intervention_urgent=intervention_urgent,
                        lane_invasion_detected=(
                            lane_invasion_detected
                        ),
                        lane_markings=lane_markings,
                        lane_keeping_enabled=lane_keeping_enabled,
                        lane_keeping_information=(
                            lane_keeping_information
                        ),
                        traffic_light_information=(
                            traffic_light_information
                        ),
                        obstacle_distance=obstacle_distance,
                        collision_detected=collision_detected
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

            if pressed_key in (
                ord("l"),
                ord("L")
            ):
                lane_keeping_enabled = (
                    not lane_keeping_enabled
                )

                print(
                    "Lane keeping assist:",
                    "enabled"
                    if lane_keeping_enabled
                    else "disabled"
                )

    finally:
        alert_manager.close()
        safety_logger.close()
        controller.deactivate(ego_vehicle)
