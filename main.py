import traceback

from carla_client import connect_to_carla
from cleanup import (
    cleanup,
    destroy_existing_vehicles
)
from config import (
    DRIVING_MODE,
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
    create_lane_invasion_sensor
)
from simulation import run_simulation
from vehicles import (
    spawn_ego_vehicle,
    spawn_traffic_vehicles
)


def create_controller(client):
    selected_mode = DRIVING_MODE.lower().strip()

    if selected_mode == "ai":
        return AIController()

    if selected_mode == "autopilot":
        return AutopilotController(client)

    raise ValueError(
        f"Unsupported driving mode: {DRIVING_MODE}"
    )


def main():
    camera = None
    obstacle_sensor = None
    collision_sensor = None
    lane_invasion_sensor = None
    sensor_list = []
    created_vehicles = []
    data_collector = None

    try:
        client, world = connect_to_carla()

        destroy_existing_vehicles(world)

        ego_vehicle = spawn_ego_vehicle(world)
        created_vehicles.append(ego_vehicle)

        traffic_vehicles = spawn_traffic_vehicles(
            world,
            NUMBER_OF_TRAFFIC_VEHICLES
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

        controller = create_controller(client)

        if COLLECTING_DATA:
            data_collector = DataCollector()
            data_collector.start()

        run_simulation(
            world=world,
            ego_vehicle=ego_vehicle,
            controller=controller,
            data_collector=data_collector
        )

    except KeyboardInterrupt:
        print("Simulation stopped by user")

    except Exception:
        print("An unexpected error occurred:")
        traceback.print_exc()

    finally:
        if data_collector is not None:
            data_collector.close()

        cleanup(
            sensor_list,
            created_vehicles
        )


if __name__ == "__main__":
    main()
