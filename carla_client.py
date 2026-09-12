import carla

from config import HOST, PORT, CLIENT_TIMEOUT, MAP_NAME


def connect_to_carla():
    print(
        f"Connecting to CARLA at {HOST}:{PORT}...",
        flush=True
    )
    client = carla.Client(HOST, PORT)
    client.set_timeout(CLIENT_TIMEOUT)

    try:
        world = client.get_world()
    except RuntimeError as error:
        raise RuntimeError(
            f"Could not connect to CARLA at {HOST}:{PORT}. "
            "Make sure CarlaUE4 is running and has finished loading. "
            f"Original error: {error}"
        ) from error

    if world.get_map().name.split("/")[-1] != MAP_NAME:
        print(f"Loading map: {MAP_NAME}", flush=True)
        world = client.load_world(MAP_NAME)

    print(f"Connected to: {world.get_map().name}", flush=True)

    return client, world
