import threading
import time

import carla
import numpy as np

from config import (
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FOV,
    CAMERA_SENSOR_TICK,
    CAMERA_LOCATION_X,
    CAMERA_LOCATION_Z,
    CAMERA_PITCH,

    OBSTACLE_SENSOR_DISTANCE,
    OBSTACLE_SENSOR_HIT_RADIUS,
    OBSTACLE_SENSOR_TICK
)


# =========================
# RGB camera state
# =========================

latest_frame = None
latest_frame_number = None
frame_lock = threading.Lock()


# =========================
# Obstacle sensor state
# =========================

latest_obstacle_distance = None
latest_obstacle_actor = None
latest_obstacle_time = None

obstacle_lock = threading.Lock()


# =========================
# Collision sensor state
# =========================

collision_detected = False
latest_collision_actor = None
latest_collision_time = None

collision_lock = threading.Lock()


# =========================
# Sensor state
# =========================

running = True


# =========================
# RGB callback
# =========================

def process_rgb_image(image):
    global latest_frame
    global latest_frame_number

    if not running:
        return

    array = np.frombuffer(
        image.raw_data,
        dtype=np.uint8
    )

    array = array.reshape(
        (image.height, image.width, 4)
    )

    # CARLA returns BGRA.
    # Removing alpha leaves BGR for OpenCV.
    frame = array[:, :, :3].copy()

    with frame_lock:
        latest_frame = frame
        latest_frame_number = image.frame


# =========================
# Obstacle callback
# =========================

def process_obstacle(event):
    global latest_obstacle_distance
    global latest_obstacle_actor
    global latest_obstacle_time

    if not running:
        return

    with obstacle_lock:
        latest_obstacle_distance = float(
            event.distance
        )

        latest_obstacle_actor = (
            event.other_actor
        )

        latest_obstacle_time = time.time()


# =========================
# Collision callback
# =========================

def process_collision(event):
    global collision_detected
    global latest_collision_actor
    global latest_collision_time

    if not running:
        return

    with collision_lock:
        collision_detected = True
        latest_collision_actor = event.other_actor
        latest_collision_time = time.time()


# =========================
# RGB getter
# =========================

def get_latest_frame():
    with frame_lock:
        if latest_frame is None:
            return None, None

        return (
            latest_frame.copy(),
            latest_frame_number
        )


# =========================
# Obstacle getter
# =========================

def get_latest_obstacle():
    with obstacle_lock:
        if latest_obstacle_time is None:
            return None, None

        time_since_detection = (
            time.time()
            - latest_obstacle_time
        )

        # If the sensor has not reported the obstacle
        # recently, assume there is currently no obstacle.
        if time_since_detection > 0.15:
            return None, None

        return (
            latest_obstacle_distance,
            latest_obstacle_actor
        )


# =========================
# Collision getter
# =========================

def get_latest_collision():
    with collision_lock:
        return (
            collision_detected,
            latest_collision_actor,
            latest_collision_time
        )


# =========================
# RGB camera creation
# =========================

def create_rgb_camera(
    world,
    ego_vehicle
):
    blueprints = (
        world.get_blueprint_library()
    )

    camera_blueprint = blueprints.find(
        "sensor.camera.rgb"
    )

    camera_blueprint.set_attribute(
        "image_size_x",
        str(CAMERA_WIDTH)
    )

    camera_blueprint.set_attribute(
        "image_size_y",
        str(CAMERA_HEIGHT)
    )

    camera_blueprint.set_attribute(
        "fov",
        str(CAMERA_FOV)
    )

    camera_blueprint.set_attribute(
        "sensor_tick",
        str(CAMERA_SENSOR_TICK)
    )

    camera_transform = carla.Transform(
        carla.Location(
            x=CAMERA_LOCATION_X,
            z=CAMERA_LOCATION_Z
        ),
        carla.Rotation(
            pitch=CAMERA_PITCH
        )
    )

    camera = world.spawn_actor(
        camera_blueprint,
        camera_transform,
        attach_to=ego_vehicle,
        attachment_type=carla.AttachmentType.Rigid
    )

    camera.listen(
        process_rgb_image
    )

    print("RGB camera created")

    return camera


# =========================
# Obstacle sensor creation
# =========================

def create_obstacle_sensor(
    world,
    ego_vehicle
):
    blueprints = (
        world.get_blueprint_library()
    )

    obstacle_blueprint = blueprints.find(
        "sensor.other.obstacle"
    )

    obstacle_blueprint.set_attribute(
        "distance",
        str(OBSTACLE_SENSOR_DISTANCE)
    )

    obstacle_blueprint.set_attribute(
        "hit_radius",
        str(OBSTACLE_SENSOR_HIT_RADIUS)
    )

    obstacle_blueprint.set_attribute(
        "sensor_tick",
        str(OBSTACLE_SENSOR_TICK)
    )

    obstacle_blueprint.set_attribute(
        "only_dynamics",
        "true"
    )

    obstacle_transform = carla.Transform(
        carla.Location(
            x=2.2,
            y=0.0,
            z=1.0
        ),
        carla.Rotation(
            pitch=0.0,
            yaw=0.0,
            roll=0.0
        )
    )

    obstacle_sensor = world.spawn_actor(
        obstacle_blueprint,
        obstacle_transform,
        attach_to=ego_vehicle,
        attachment_type=carla.AttachmentType.Rigid
    )

    obstacle_sensor.listen(
        process_obstacle
    )

    print("Obstacle sensor created")

    return obstacle_sensor


# =========================
# Collision sensor creation
# =========================

def create_collision_sensor(
    world,
    ego_vehicle
):
    blueprints = (
        world.get_blueprint_library()
    )

    collision_blueprint = blueprints.find(
        "sensor.other.collision"
    )

    collision_transform = carla.Transform(
        carla.Location(
            x=0.0,
            y=0.0,
            z=0.0
        )
    )

    collision_sensor = world.spawn_actor(
        collision_blueprint,
        collision_transform,
        attach_to=ego_vehicle,
        attachment_type=carla.AttachmentType.Rigid
    )

    collision_sensor.listen(
        process_collision
    )

    print("Collision sensor created")

    return collision_sensor


# =========================
# Stop sensors
# =========================

def stop_sensors():
    global running

    running = False