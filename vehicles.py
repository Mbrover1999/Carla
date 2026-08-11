import random

from config import EGO_VEHICLE_BLUEPRINT


def spawn_ego_vehicle(world):
    blueprints = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    ego_blueprint = blueprints.find(
        EGO_VEHICLE_BLUEPRINT
    )

    shuffled_spawn_points = spawn_points.copy()
    random.shuffle(shuffled_spawn_points)

    for spawn_point in shuffled_spawn_points:
        ego_vehicle = world.try_spawn_actor(
            ego_blueprint,
            spawn_point
        )

        if ego_vehicle is not None:
            print(
                "Ego vehicle spawned:",
                ego_vehicle.type_id
            )

            return ego_vehicle

    raise RuntimeError(
        "Could not spawn the ego vehicle"
    )


def spawn_traffic_vehicles(world, amount):
    blueprints = world.get_blueprint_library()
    vehicle_blueprints = blueprints.filter("vehicle.*")
    spawn_points = world.get_map().get_spawn_points()

    traffic_vehicles = []

    shuffled_spawn_points = spawn_points.copy()
    random.shuffle(shuffled_spawn_points)

    for spawn_point in shuffled_spawn_points:
        if len(traffic_vehicles) >= amount:
            break

        vehicle_blueprint = random.choice(
            vehicle_blueprints
        )

        vehicle = world.try_spawn_actor(
            vehicle_blueprint,
            spawn_point
        )

        if vehicle is None:
            continue

        vehicle.set_autopilot(True)
        traffic_vehicles.append(vehicle)

        print(
            "Traffic vehicle spawned:",
            vehicle.type_id
        )

    print(
        f"Created {len(traffic_vehicles)} "
        f"out of {amount} requested traffic vehicles"
    )

    return traffic_vehicles