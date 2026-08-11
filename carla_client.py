import carla

from config import HOST, PORT, CLIENT_TIMEOUT, MAP_NAME


def connect_to_carla():
    client = carla.Client(HOST, PORT)
    client.set_timeout(CLIENT_TIMEOUT)

    world = client.get_world()
    if world.get_map().name.split("/")[-1] != MAP_NAME:
        print(f"Loading map: {MAP_NAME}")
        world = client.load_world(MAP_NAME)

    print(f"Connected to: {world.get_map().name}")

    return client, world