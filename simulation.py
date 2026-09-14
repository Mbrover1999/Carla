import json
import math
import time
from datetime import datetime

import carla
import cv2

import sensors

from config import (
    CONTROLLER_INACTIVITY_ENABLED,
    BLIND_SPOT_DETECTION_ENABLED,
    CROSS_TRAFFIC_DETECTION_ENABLED,
    CUT_IN_DETECTION_ENABLED,
    INTERSECTION_CONTROLLER_ENABLED,
    LANE_INVASION_ENABLED,
    LANE_KEEPING_ENABLED,
    NAVIGATION_ENABLED,
    PROJECT_ROOT,
    ROAD_SPEED_CONTROL_ENABLED,
    RUN_DURATION_SECONDS,
    SAFETY_ENABLED,
    STOP_SIGN_DETECTION_ENABLED,
    TRAFFIC_LIGHT_DETECTION_ENABLED
)
from journey_evaluator import JourneyEvaluator
from navigation.intersection_controller import (
    IntersectionController
)
from navigation.road_speed_controller import (
    RoadSpeedController
)
from navigation.route_manager import RouteManager
from navigation.turn_signals import (
    HAZARD,
    LEFT,
    OFF,
    RIGHT,
    select_turn_signal
)
from safety.inactivity_detector import (
    ControllerInactivityDetector
)
from safety.alert_manager import SafetyAlertManager
from safety.blind_spot_safety import BlindSpotSafety
from safety.cross_traffic_safety import CrossTrafficSafety
from safety.cut_in_safety import CutInSafety
from safety.emergency_pull_over import EmergencyPullOverController
from safety.lane_keeping import LaneKeepingAssist
from safety.safety_layer import SafetyLayer
from safety.safety_logger import SafetyLogger
from safety.stop_sign_safety import StopSignSafety
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


def set_vehicle_indicators(vehicle, signal=OFF):
    try:
        current_state = int(vehicle.get_light_state())
        indicator_flags = int(
            carla.VehicleLightState.LeftBlinker
            | carla.VehicleLightState.RightBlinker
        )
        updated_state = current_state & ~indicator_flags

        if signal in (LEFT, HAZARD):
            updated_state |= int(
                carla.VehicleLightState.LeftBlinker
            )

        if signal in (RIGHT, HAZARD):
            updated_state |= int(
                carla.VehicleLightState.RightBlinker
            )

        vehicle.set_light_state(
            carla.VehicleLightState(updated_state)
        )
        return True
    except (AttributeError, RuntimeError, TypeError, ValueError):
        return False


def set_hazard_lights(vehicle, enabled):
    return set_vehicle_indicators(
        vehicle,
        HAZARD if enabled else OFF
    )


def draw_controller_information(
    frame,
    information,
    obstacle_safety_state="DISABLED",
    inactivity_state="DISABLED",
    inactivity_seconds=0.0,
    inactivity_test_active=False,
    emergency_information=None,
    intervention_reason=None,
    intervention_urgent=False,
    lane_invasion_detected=False,
    lane_markings=None,
    lane_keeping_enabled=False,
    lane_keeping_information=None,
    traffic_light_information=None,
    stop_sign_information=None,
    cross_traffic_information=None,
    cut_in_information=None,
    blind_spot_information=None,
    navigation_information=None,
    intersection_information=None,
    road_speed_information=None,
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

    if road_speed_information is not None:
        lines.append(
            "Road limit / target: "
            f"{road_speed_information['speed_limit_kmh']:.0f} / "
            f"{road_speed_information['target_speed_kmh']:.0f} km/h"
        )

    if navigation_information is not None:
        maneuver = navigation_information["maneuver"]
        maneuver_distance = navigation_information.get(
            "maneuver_distance_m"
        )
        maneuver_text = maneuver

        if maneuver_distance is not None:
            maneuver_text += f" in {maneuver_distance:.0f} m"

        lines.append(
            f"Navigation: {navigation_information['mode']} | "
            f"{maneuver_text}"
        )

        lines.append(
            "Destination: "
            f"{navigation_information['destination_distance_m']:.0f} m "
            f"(route {navigation_information['route_number']}) "
            "[N: new]"
        )

    if (
        intersection_information is not None
        and intersection_information["active"]
    ):
        lines.append(
            "Route steering: "
            f"{intersection_information['applied_steering']:+.3f} "
            f"(error "
            f"{intersection_information['heading_error_deg']:+.1f} deg)"
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

    if (
        emergency_information is not None
        and emergency_information["phase"]
        != EmergencyPullOverController.INACTIVE
    ):
        lines.append(
            "Emergency response: "
            f"{emergency_information['phase']}"
        )

        if emergency_information["call_started"]:
            lines.append("Emergency call: SIMULATED 911 CALL")
        else:
            call_countdown = emergency_information.get(
                "call_countdown_seconds"
            )

            if call_countdown is not None:
                lines.append(
                    "Simulated 911 call in: "
                    f"{call_countdown:.0f} s"
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

    stop_sign_state = (
        stop_sign_information["safety_state"]
        if stop_sign_information is not None
        else StopSignSafety.DISABLED
    )
    stop_sign_text = stop_sign_state

    if (
        stop_sign_information is not None
        and stop_sign_information.get("distance_m") is not None
    ):
        stop_sign_text += (
            f" ({stop_sign_information['distance_m']:.1f} m)"
        )

    if (
        stop_sign_information is not None
        and stop_sign_information.get("hold_remaining_s") is not None
    ):
        stop_sign_text += (
            f" [{stop_sign_information['hold_remaining_s']:.1f} s]"
        )

    if (
        stop_sign_information is not None
        and stop_sign_information.get(
            "waiting_for_cross_traffic",
            False
        )
    ):
        stop_sign_text += " [WAITING FOR CROSS TRAFFIC]"

    lines.append(f"Stop sign: {stop_sign_text}")

    if cross_traffic_information is None:
        cross_traffic_state = CrossTrafficSafety.DISABLED
        cross_traffic_distance = None
        cross_traffic_gap = None
    else:
        cross_traffic_state = cross_traffic_information[
            "safety_state"
        ]
        cross_traffic_distance = cross_traffic_information.get(
            "distance_m"
        )
        cross_traffic_gap = cross_traffic_information.get(
            "arrival_gap_s"
        )

    cross_traffic_text = cross_traffic_state

    if cross_traffic_distance is not None:
        cross_traffic_text += f" ({cross_traffic_distance:.1f} m"

        if cross_traffic_gap is not None:
            cross_traffic_text += f", gap {cross_traffic_gap:.1f} s"

        cross_traffic_text += ")"

    lines.append(f"Cross traffic: {cross_traffic_text}")

    cut_in_state = (
        cut_in_information["safety_state"]
        if cut_in_information is not None
        else CutInSafety.DISABLED
    )
    cut_in_text = cut_in_state

    if (
        cut_in_information is not None
        and cut_in_information.get("intrusion_time_s") is not None
    ):
        cut_in_text += (
            f" ({cut_in_information['side']}, "
            f"{cut_in_information['intrusion_time_s']:.1f}s)"
        )

    lines.append(f"Cut-in prediction: {cut_in_text}")

    blind_spot_state = (
        blind_spot_information["state"]
        if blind_spot_information is not None
        else "DISABLED"
    )
    lines.append(f"Blind spot: {blind_spot_state}")

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
    traffic_light_state="CLEAR",
    stop_sign_state="CLEAR",
    cross_traffic_state="CLEAR",
    cut_in_state="CLEAR",
    blind_spot_intervention_state="CLEAR"
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

    if stop_sign_state not in (
        StopSignSafety.CLEAR,
        StopSignSafety.RELEASED,
        StopSignSafety.DISABLED
    ):
        active_states.append(stop_sign_state)

    if cross_traffic_state not in (
        CrossTrafficSafety.CLEAR,
        CrossTrafficSafety.DISABLED
    ):
        active_states.append(cross_traffic_state)

    if cut_in_state not in (
        CutInSafety.CLEAR,
        CutInSafety.DISABLED
    ):
        active_states.append(cut_in_state)

    if blind_spot_intervention_state != "CLEAR":
        active_states.append(blind_spot_intervention_state)

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
    traffic_light_state=None,
    stop_sign_state=None,
    cross_traffic_state=None,
    cut_in_state=None,
    blind_spot_intervention_state=None
):
    reasons = []
    urgent = False

    obstacle_reasons = {
        "CREEPING": "Obstacle ahead - moving carefully",
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
            "Controller inactive - emergency pull-over"
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
        ),
        TrafficLightSafety.RED_CREEPING: (
            "Red traffic light - approaching stop line"
        )
    }

    traffic_light_reason = traffic_light_reasons.get(
        traffic_light_state
    )

    if traffic_light_reason is not None:
        reasons.append(traffic_light_reason)

    if traffic_light_state == TrafficLightSafety.RED_BRAKING:
        urgent = True

    stop_sign_reasons = {
        StopSignSafety.APPROACHING: "Stop sign ahead - slowing down",
        StopSignSafety.BRAKING: "Stop sign ahead - braking",
        StopSignSafety.CREEPING: "Stop sign - approaching stop line",
        StopSignSafety.HOLDING: "Stop sign - complete stop"
    }
    stop_sign_reason = stop_sign_reasons.get(stop_sign_state)

    if stop_sign_reason is not None:
        reasons.append(stop_sign_reason)

    if stop_sign_state in (
        StopSignSafety.BRAKING,
        StopSignSafety.HOLDING
    ):
        urgent = True

    cross_traffic_reasons = {
        CrossTrafficSafety.WARNING: (
            "Cross traffic approaching"
        ),
        CrossTrafficSafety.BRAKING: (
            "Cross traffic conflict - stopping"
        )
    }
    cross_traffic_reason = cross_traffic_reasons.get(
        cross_traffic_state
    )

    if cross_traffic_reason is not None:
        reasons.append(cross_traffic_reason)

    if cross_traffic_state == CrossTrafficSafety.BRAKING:
        urgent = True

    cut_in_reasons = {
        CutInSafety.WARNING: "Vehicle merging into lane - slowing down",
        CutInSafety.BRAKING: "Vehicle cutting in - braking",
        CutInSafety.EMERGENCY: (
            "Vehicle cutting in - EMERGENCY BRAKING"
        )
    }
    cut_in_reason = cut_in_reasons.get(cut_in_state)

    if cut_in_reason is not None:
        reasons.append(cut_in_reason)

    if cut_in_state in (CutInSafety.BRAKING, CutInSafety.EMERGENCY):
        urgent = True

    if blind_spot_intervention_state == "BLIND_SPOT_RIGHT_BLOCKED":
        reasons.append("Vehicle in right blind spot - lane change blocked")
        urgent = True

    if blind_spot_intervention_state == "BLIND_SPOT_LEFT_BLOCKED":
        reasons.append("Vehicle in left blind spot - lane change blocked")
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
    data_collector=None,
    run_duration_seconds=RUN_DURATION_SECONDS,
    scenario_runtime=None,
    control_command_source=None,
    evaluate_journey=False,
    stop_requested=None
):
    controller.activate(ego_vehicle)
    safety_layer = SafetyLayer()
    inactivity_detector = ControllerInactivityDetector()
    lane_keeping_assist = LaneKeepingAssist(
        world.get_map()
    )
    traffic_light_safety = TrafficLightSafety()
    stop_sign_safety = StopSignSafety()
    cross_traffic_safety = CrossTrafficSafety()
    cut_in_safety = CutInSafety()
    blind_spot_safety = BlindSpotSafety()
    emergency_pull_over = EmergencyPullOverController()
    route_manager = (
        RouteManager(
            world_map=world.get_map(),
            vehicle=ego_vehicle
        )
        if NAVIGATION_ENABLED
        else None
    )
    intersection_controller = IntersectionController()
    road_speed_controller = RoadSpeedController()
    alert_manager = SafetyAlertManager()
    free_drive_log_path = None

    if evaluate_journey:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        free_drive_log_path = (
            PROJECT_ROOT
            / "safety_logs"
            / f"free_drive_{timestamp}.csv"
        )

    safety_logger = SafetyLogger(log_path=free_drive_log_path)
    safety_logger.start()
    journey_evaluator = JourneyEvaluator() if evaluate_journey else None

    end_time = time.time() + run_duration_seconds
    simulation_start_time = time.time()
    last_processed_frame_number = None
    controller_information = None
    requested_control = None
    obstacle_safety_state = "DISABLED"
    inactivity_state = "DISABLED"
    inactivity_seconds = 0.0
    inactivity_test_active = False
    emergency_information = (
        EmergencyPullOverController.information()
    )
    indicator_signal = OFF
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
    stop_sign_information = StopSignSafety.information(
        safety_state=StopSignSafety.DISABLED
    )
    cross_traffic_information = CrossTrafficSafety.information(
        safety_state=CrossTrafficSafety.DISABLED
    )
    cut_in_information = CutInSafety.information(
        safety_state=CutInSafety.DISABLED
    )
    blind_spot_information = BlindSpotSafety.information()
    blind_spot_intervention_state = "CLEAR"
    navigation_information = None
    intersection_information = (
        IntersectionController.information()
    )
    road_speed_information = None
    safety_event_key = None
    last_console_event_state = None
    termination_reason = "COMPLETED"
    scenario_elapsed_seconds = 0.0
    scenario_inactivity_active = False
    scenario_lane_keeping_suppressed = False
    scenario_cross_traffic_active = False

    try:
        while time.time() < end_time:
            if (
                stop_requested is not None
                and stop_requested()
            ):
                termination_reason = "STOPPED"
                break

            world.wait_for_tick()

            scenario_elapsed_seconds = (
                time.time() - simulation_start_time
            )

            if scenario_runtime is not None:
                scenario_runtime.update(
                    scenario_elapsed_seconds
                )
                scenario_inactivity_active = (
                    scenario_runtime.controller_inactive(
                        scenario_elapsed_seconds
                    )
                )
                scenario_lane_keeping_suppressed = (
                    scenario_runtime.suppress_lane_keeping(
                        scenario_elapsed_seconds
                    )
                )
                scenario_cross_traffic_active = (
                    scenario_runtime.force_cross_traffic_detection(
                        scenario_elapsed_seconds
                    )
                )

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
                    not (
                        inactivity_test_active
                        or scenario_inactivity_active
                    )
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

                navigation_mode = RouteManager.AI

                if route_manager is not None:
                    navigation_information = (
                        route_manager.update()
                    )
                    navigation_mode = navigation_information[
                        "mode"
                    ]

                intersection_active = (
                    controller_responded
                    and INTERSECTION_CONTROLLER_ENABLED
                    and navigation_information is not None
                    and navigation_mode in (
                        RouteManager.APPROACH,
                        RouteManager.INTERSECTION
                    )
                )

                if intersection_active:
                    (
                        requested_control,
                        intersection_information
                    ) = intersection_controller.apply(
                        vehicle=ego_vehicle,
                        requested_control=requested_control,
                        target_waypoint=(
                            navigation_information[
                                "target_waypoint"
                            ]
                        )
                    )
                else:
                    intersection_information = (
                        IntersectionController.information()
                    )

                    if controller_responded:
                        intersection_controller.reset(
                            steering=requested_control.steer
                        )

                if (
                    controller_responded
                    and ROAD_SPEED_CONTROL_ENABLED
                ):
                    (
                        requested_control,
                        road_speed_information
                    ) = road_speed_controller.apply(
                        vehicle=ego_vehicle,
                        requested_control=requested_control,
                        current_speed_kmh=speed_kmh,
                        navigation_mode=navigation_mode,
                        maneuver=(
                            navigation_information["maneuver"]
                            if navigation_information is not None
                            else None
                        )
                    )

                if (
                    scenario_runtime is not None
                    and requested_control is not None
                ):
                    requested_control = (
                        scenario_runtime.apply_requested_control(
                            scenario_elapsed_seconds,
                            requested_control
                        )
                    )

                (
                    obstacle_distance,
                    obstacle_actor
                ) = sensors.get_latest_obstacle()

                if (
                    obstacle_distance is not None
                    and not safety_layer.is_obstacle_relevant(
                        ego_vehicle=ego_vehicle,
                        obstacle_actor=obstacle_actor,
                        world_map=world.get_map()
                    )
                ):
                    obstacle_distance = None

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

                if scenario_runtime is not None:
                    scenario_runtime.notify_lane_invasion(
                        lane_invasion_detected
                    )

                if BLIND_SPOT_DETECTION_ENABLED:
                    blind_spot_information = blind_spot_safety.inspect(
                        world=world,
                        ego_vehicle=ego_vehicle,
                        radar_readings=(
                            sensors.get_blind_spot_radar_readings()
                        )
                    )
                else:
                    blind_spot_information = BlindSpotSafety.information()

                cut_in_information = cut_in_safety.inspect(
                    world=world,
                    world_map=world.get_map(),
                    ego_vehicle=ego_vehicle,
                    active=(
                        SAFETY_ENABLED
                        and CUT_IN_DETECTION_ENABLED
                    )
                )

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

                final_control = cut_in_safety.apply(
                    requested_control=final_control,
                    information=cut_in_information
                )

                cross_traffic_active = (
                    SAFETY_ENABLED
                    and CROSS_TRAFFIC_DETECTION_ENABLED
                    and navigation_information is not None
                    and (
                        scenario_cross_traffic_active
                        or navigation_mode in (
                            RouteManager.APPROACH,
                            RouteManager.INTERSECTION
                        )
                    )
                )

                cross_traffic_information = (
                    cross_traffic_safety.inspect(
                        world=world,
                        ego_vehicle=ego_vehicle,
                        active=cross_traffic_active,
                        target_waypoint=(
                            navigation_information[
                                "target_waypoint"
                            ]
                            if navigation_information is not None
                            else None
                        )
                    )
                )

                final_control = cross_traffic_safety.apply(
                    requested_control=final_control,
                    information=cross_traffic_information,
                    speed_kmh=speed_kmh
                )

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

                stop_sign_information = stop_sign_safety.inspect(
                    world=world,
                    world_map=world.get_map(),
                    ego_vehicle=ego_vehicle,
                    speed_kmh=speed_kmh,
                    cross_traffic_blocked=(
                        cross_traffic_information[
                            "safety_state"
                        ] in (
                            CrossTrafficSafety.WARNING,
                            CrossTrafficSafety.BRAKING
                        )
                    ),
                    active=(
                        SAFETY_ENABLED
                        and STOP_SIGN_DETECTION_ENABLED
                    )
                )
                final_control = stop_sign_safety.apply(
                    requested_control=final_control,
                    information=stop_sign_information,
                    speed_kmh=speed_kmh
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
                    and controller_responded
                    and lane_keeping_enabled
                    and not scenario_lane_keeping_suppressed
                    and navigation_mode == RouteManager.AI
                    and inactivity_state
                    != ControllerInactivityDetector.SAFE_STOP
                    and obstacle_safety_state not in (
                        "BRAKING",
                        "EMERGENCY"
                    )
                    and cut_in_information[
                        "safety_state"
                    ] not in (
                        CutInSafety.BRAKING,
                        CutInSafety.EMERGENCY
                    )
                    and cross_traffic_information[
                        "safety_state"
                    ] != CrossTrafficSafety.BRAKING
                    and traffic_light_information[
                        "safety_state"
                    ] != TrafficLightSafety.RED_BRAKING
                    and stop_sign_information[
                        "safety_state"
                    ] not in (
                        StopSignSafety.BRAKING,
                        StopSignSafety.HOLDING
                    )
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
                    (
                        final_control,
                        emergency_information
                    ) = emergency_pull_over.apply(
                        vehicle=ego_vehicle,
                        requested_control=final_control,
                        world_map=world.get_map(),
                        world=world,
                        speed_kmh=speed_kmh,
                        inactive_seconds=inactivity_seconds,
                        blind_spot_information=blind_spot_information
                    )
                else:
                    emergency_was_active = emergency_pull_over.active
                    emergency_pull_over.reset()
                    emergency_information = (
                        EmergencyPullOverController.information()
                    )

                    if emergency_was_active:
                        alert_manager.stop_current_sound()

                blind_spot_intervention_state = "CLEAR"

                if (
                    emergency_information["phase"]
                    == EmergencyPullOverController.WAITING_FOR_RIGHT_LANE
                    and blind_spot_information["right_occupied"]
                ):
                    blind_spot_intervention_state = (
                        "BLIND_SPOT_RIGHT_BLOCKED"
                    )

                requested_hazards = emergency_information[
                    "hazards_active"
                ]
                requested_indicator = select_turn_signal(
                    navigation_information,
                    hazards_active=requested_hazards
                )

                if requested_indicator != indicator_signal:
                    if set_vehicle_indicators(
                        ego_vehicle,
                        requested_indicator
                    ):
                        indicator_signal = requested_indicator
                        print(
                            f"TURN_SIGNAL: {indicator_signal}",
                            flush=True
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
                    traffic_light_information["safety_state"],
                    stop_sign_information["safety_state"],
                    cross_traffic_information["safety_state"],
                    cut_in_information["safety_state"],
                    blind_spot_intervention_state
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
                    traffic_light_event_key,
                    stop_sign_information["event_key"],
                    cross_traffic_information["event_key"],
                    cut_in_information["event_key"],
                    (
                        blind_spot_information["event_key"]
                        if blind_spot_intervention_state != "CLEAR"
                        else None
                    )
                )

                alert_manager.update(
                    reason=intervention_reason,
                    urgent=intervention_urgent,
                    event_key=safety_event_key
                )

                if emergency_information["call_requested"]:
                    alert_manager.play_call_start()
                elif emergency_information["horn_requested"]:
                    alert_manager.play_horn()

                ego_vehicle.apply_control(
                    final_control
                )

                current_safety_state = combine_safety_states(
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
                    ],
                    stop_sign_information["safety_state"],
                    cross_traffic_information[
                        "safety_state"
                    ],
                    cut_in_information["safety_state"],
                    blind_spot_intervention_state
                )

                if current_safety_state != last_console_event_state:
                    print(
                        f"CURRENT_EVENT: {current_safety_state}",
                        flush=True
                    )
                    last_console_event_state = current_safety_state

                safety_logger.log_event(
                    speed_kmh=speed_kmh,
                    obstacle_distance=obstacle_distance,
                    safety_state=current_safety_state,
                    control=final_control,
                    collision=collision_detected,
                    collision_actor=collision_actor,
                    event_key=safety_event_key
                )

                if journey_evaluator is not None:
                    journey_evaluator.update(
                        location=ego_vehicle.get_location(),
                        speed_kmh=speed_kmh,
                        target_speed_kmh=(
                            road_speed_information[
                                "target_speed_kmh"
                            ]
                            if road_speed_information is not None
                            else None
                        ),
                        safety_state=current_safety_state,
                        collision=collision_detected,
                        lane_event_key=lane_event_key
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
                    and not scenario_inactivity_active
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
                            or scenario_inactivity_active
                        ),
                        emergency_information=(
                            emergency_information
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
                        stop_sign_information=stop_sign_information,
                        cross_traffic_information=(
                            cross_traffic_information
                        ),
                        cut_in_information=cut_in_information,
                        blind_spot_information=(
                            blind_spot_information
                        ),
                        navigation_information=(
                            navigation_information
                        ),
                        intersection_information=(
                            intersection_information
                        ),
                        road_speed_information=(
                            road_speed_information
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
            external_commands = (
                control_command_source()
                if control_command_source is not None
                else []
            )

            if pressed_key in (
                ord("q"),
                27
            ):
                termination_reason = "STOPPED"
                break

            if (
                pressed_key in (
                    ord("i"),
                    ord("I")
                )
                or "TOGGLE_INACTIVITY" in external_commands
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

            if (
                pressed_key in (ord("l"), ord("L"))
                or "TOGGLE_LANE_KEEPING" in external_commands
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

            if (
                route_manager is not None
                and (
                    pressed_key in (ord("n"), ord("N"))
                    or "NEW_ROUTE" in external_commands
                )
            ):
                route_manager.plan_new_route()
                intersection_controller.reset()

        if journey_evaluator is not None:
            journey_result = journey_evaluator.result(
                duration_seconds=time.time() - simulation_start_time,
                termination_reason=termination_reason
            )
            print(
                "JOURNEY_RESULT: "
                + json.dumps(journey_result, separators=(",", ":")),
                flush=True
            )
            print(
                f"FREE_DRIVE_LOG: {safety_logger.log_path.resolve()}",
                flush=True
            )

        return termination_reason

    finally:
        shutdown_actions = []

        if indicator_signal != OFF:
            shutdown_actions.append((
                "turn signals",
                lambda: set_vehicle_indicators(ego_vehicle, OFF)
            ))

        shutdown_actions.extend([
            ("alert manager", alert_manager.close),
            ("safety logger", safety_logger.close),
            (
                "controller",
                lambda: controller.deactivate(ego_vehicle)
            )
        ])

        for component_name, shutdown_action in shutdown_actions:
            try:
                shutdown_action()
            except Exception as error:
                print(
                    f"Shutdown warning ({component_name}): {error!r}",
                    flush=True
                )
