import argparse
import traceback
from pathlib import Path

from carla_client import connect_to_carla
from cleanup import (
    cleanup,
    destroy_existing_vehicles
)
from config import (
    DRIVING_MODE,
    RUN_DURATION_SECONDS,
    NUMBER_OF_TRAFFIC_VEHICLES,
    COLLECTING_DATA
)
from controllers.ai_controller import AIController
from controllers.autopilot_controller import (
    AutopilotController
)
from data_collector import DataCollector
from sensors import (
    create_rgb_camera,
    create_obstacle_sensor,
    create_collision_sensor,
    create_lane_invasion_sensor,
    create_blind_spot_radars
)
from scenario_catalog import (
    SCENARIOS,
    SimulationSettings
)
from scenarios import setup_scenario
from simulation import run_simulation
from vehicles import (
    spawn_ego_vehicle,
    spawn_traffic_vehicles
)


def create_control_command_reader(command_file):
    position = 0

    def read_commands():
        nonlocal position

        if command_file is None or not command_file.exists():
            return []

        try:
            with command_file.open("r", encoding="utf-8") as stream:
                stream.seek(position)
                content = stream.read()
                position = stream.tell()
                commands = [
                    line.strip()
                    for line in content.splitlines()
                    if line.strip()
                ]
                return commands
        except OSError:
            return []

    return read_commands


def create_controller(client):
    selected_mode = DRIVING_MODE.lower().strip()

    if selected_mode == "ai":
        return AIController()

    if selected_mode == "autopilot":
        return AutopilotController(client)

    raise ValueError(
        f"Unsupported driving mode: {DRIVING_MODE}"
    )


def main(
    run_duration_seconds=RUN_DURATION_SECONDS,
    traffic_vehicle_count=NUMBER_OF_TRAFFIC_VEHICLES,
    scenario_id="free_drive",
    stop_request_file=None,
    control_command_file=None
):
    camera = None
    obstacle_sensor = None
    collision_sensor = None
    lane_invasion_sensor = None
    blind_spot_radars = []
    sensor_list = []
    created_vehicles = []
    data_collector = None
    scenario_runtime = None

    try:
        print(f"Starting scenario: {scenario_id}")
        print(
            "Simulation settings: "
            f"duration={run_duration_seconds}s, "
            f"traffic_vehicles={traffic_vehicle_count}"
        )

        client, world = connect_to_carla()

        destroy_existing_vehicles(world)

        ego_vehicle = spawn_ego_vehicle(world)
        created_vehicles.append(ego_vehicle)

        scenario_runtime = setup_scenario(
            scenario_id,
            world,
            ego_vehicle
        )
        created_vehicles.extend(scenario_runtime.actors)

        traffic_vehicles = spawn_traffic_vehicles(
            world,
            traffic_vehicle_count
        )

        created_vehicles.extend(
            traffic_vehicles
        )

        camera = create_rgb_camera(
            world,
            ego_vehicle
        )
        sensor_list.append(camera)

        obstacle_sensor = create_obstacle_sensor(
            world,
            ego_vehicle
        )
        sensor_list.append(obstacle_sensor)

        collision_sensor = create_collision_sensor(
            world,
            ego_vehicle
        )
        sensor_list.append(collision_sensor)

        lane_invasion_sensor = (
            create_lane_invasion_sensor(
                world,
                ego_vehicle
            )
        )
        sensor_list.append(lane_invasion_sensor)

        try:
            blind_spot_radars = create_blind_spot_radars(
                world,
                ego_vehicle
            )
            sensor_list.extend(blind_spot_radars)
        except (AttributeError, RuntimeError) as error:
            # An unavailable radar blueprint must not prevent the rest of
            # the simulator and its safety layers from starting.
            print(
                "WARNING: Blind-spot radars are unavailable; "
                f"continuing with CARLA vehicle tracking. ({error})",
                flush=True
            )

        controller = create_controller(client)

        if COLLECTING_DATA:
            data_collector = DataCollector()
            data_collector.start()

        simulation_result = run_simulation(
            world=world,
            ego_vehicle=ego_vehicle,
            controller=controller,
            data_collector=data_collector,
            run_duration_seconds=run_duration_seconds,
            scenario_runtime=scenario_runtime,
            evaluate_journey=(scenario_id == "free_drive"),
            control_command_source=create_control_command_reader(
                control_command_file
            ),
            stop_requested=(
                lambda: stop_request_file.exists()
                if stop_request_file is not None
                else False
            )
        )

        print(
            f"SIMULATION_RESULT: {simulation_result}",
            flush=True
        )

        return 0

    except KeyboardInterrupt:
        print("Simulation stopped by user")
        print("SIMULATION_RESULT: STOPPED", flush=True)
        return 0

    except Exception:
        print("An unexpected error occurred:")
        traceback.print_exc()
        print("SIMULATION_RESULT: ERROR", flush=True)
        return 1

    finally:
        try:
            if scenario_runtime is not None:
                scenario_runtime.close()

            if data_collector is not None:
                data_collector.close()

            cleanup(
                sensor_list,
                created_vehicles
            )
        except Exception:
            # Cleanup problems should be visible, but they must not turn a
            # successfully completed simulation into a failed run.
            print("Cleanup completed with a warning:")
            traceback.print_exc()


def parse_arguments(arguments=None):
    parser = argparse.ArgumentParser(
        description="Run the CARLA autonomous-driving simulation"
    )
    parser.add_argument(
        "--scenario",
        choices=[scenario.scenario_id for scenario in SCENARIOS],
        default="free_drive"
    )
    parser.add_argument(
        "--duration-minutes",
        type=int,
        default=max(1, RUN_DURATION_SECONDS // 60)
    )
    parser.add_argument(
        "--traffic-vehicles",
        type=int,
        default=NUMBER_OF_TRAFFIC_VEHICLES
    )
    parser.add_argument(
        "--stop-request-file",
        type=Path,
        default=None,
        help=argparse.SUPPRESS
    )
    parser.add_argument(
        "--control-command-file",
        type=Path,
        default=None,
        help=argparse.SUPPRESS
    )

    return parser.parse_args(arguments)


def run_from_command_line(arguments=None):
    arguments = parse_arguments(arguments)
    settings = SimulationSettings(
        scenario_id=arguments.scenario,
        duration_minutes=arguments.duration_minutes,
        traffic_vehicles=arguments.traffic_vehicles
    )

    try:
        settings.validate()
    except ValueError as error:
        print(f"Invalid simulation settings: {error}")
        return 2

    return main(
        run_duration_seconds=settings.duration_seconds,
        traffic_vehicle_count=settings.traffic_vehicles,
        scenario_id=settings.scenario_id,
        stop_request_file=arguments.stop_request_file,
        control_command_file=arguments.control_command_file
    )


if __name__ == "__main__":
    raise SystemExit(run_from_command_line())
