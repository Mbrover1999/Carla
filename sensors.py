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
    OBSTACLE_SENSOR_TICK,

    BLIND_SPOT_RADAR_RANGE,
    BLIND_SPOT_RADAR_HORIZONTAL_FOV,
    BLIND_SPOT_RADAR_VERTICAL_FOV,
    BLIND_SPOT_RADAR_POINTS_PER_SECOND,
    BLIND_SPOT_RADAR_SENSOR_TICK,
    BLIND_SPOT_READING_MAX_AGE_SECONDS,

    LANE_INVASION_ALERT_DURATION_SECONDS
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
# Lane invasion sensor state
# =========================

latest_lane_markings = []
latest_lane_invasion_time = None
latest_lane_invasion_frame = None

lane_invasion_lock = threading.Lock()


# =========================
# Blind-spot radar state
# =========================

blind_spot_readings = {
    "left": None,
    "right": None
}
blind_spot_lock = threading.Lock()


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
# Lane invasion callback
# =========================

def process_lane_invasion(event):
    global latest_lane_markings
    global latest_lane_invasion_time
    global latest_lane_invasion_frame

    if not running:
        return

    marking_names = []

    for marking in event.crossed_lane_markings:
        marking_type = getattr(
            marking,
            "type",
            "Unknown"
        )

        marking_name = getattr(
            marking_type,
            "name",
            None
        )

        if marking_name is None:
            marking_name = str(marking_type).split(".")[-1]

        marking_names.append(marking_name)

    with lane_invasion_lock:
        latest_lane_markings = sorted(set(marking_names))
        latest_lane_invasion_time = time.time()
        latest_lane_invasion_frame = getattr(
            event,
            "frame",
            None
        )


def process_blind_spot_radar(measurement, side):
    """Keep the nearest radar return for each rear-side sensor."""
    if not running or side not in blind_spot_readings:
        return

    detections = list(measurement)
    nearest_depth = min(
        (float(detection.depth) for detection in detections),
        default=None
    )
    nearest_velocity = None

    if nearest_depth is not None:
        nearest = min(detections, key=lambda item: float(item.depth))
        nearest_velocity = float(nearest.velocity)

    with blind_spot_lock:
        blind_spot_readings[side] = {
            "depth_m": nearest_depth,
            "relative_velocity_mps": nearest_velocity,
            "detection_count": len(detections),
            "frame": getattr(measurement, "frame", None),
            "timestamp": time.time()
        }


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
# Lane invasion getter
# =========================

def get_latest_lane_invasion():
    with lane_invasion_lock:
        if latest_lane_invasion_time is None:
            return False, [], None

        time_since_invasion = (
            time.time()
            - latest_lane_invasion_time
        )

        if time_since_invasion > (
            LANE_INVASION_ALERT_DURATION_SECONDS
        ):
            return False, [], None

        event_key = (
            latest_lane_invasion_frame
            if latest_lane_invasion_frame is not None
            else latest_lane_invasion_time
        )

        return (
            True,
            list(latest_lane_markings),
            event_key
        )


def get_blind_spot_radar_readings():
    now = time.time()
    result = {}

    with blind_spot_lock:
        for side, reading in blind_spot_readings.items():
            if (
                reading is None
                or now - reading["timestamp"]
                > BLIND_SPOT_READING_MAX_AGE_SECONDS
            ):
                result[side] = None
            else:
                result[side] = dict(reading)

    return result


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
# Lane invasion sensor creation
# =========================

def create_lane_invasion_sensor(
    world,
    ego_vehicle
):
    blueprints = (
        world.get_blueprint_library()
    )

    lane_invasion_blueprint = blueprints.find(
        "sensor.other.lane_invasion"
    )

    lane_invasion_sensor = world.spawn_actor(
        lane_invasion_blueprint,
        carla.Transform(),
        attach_to=ego_vehicle,
        attachment_type=carla.AttachmentType.Rigid
    )

    lane_invasion_sensor.listen(
        process_lane_invasion
    )

    print("Lane invasion sensor created")

    return lane_invasion_sensor


def create_blind_spot_radars(world, ego_vehicle):
    """Attach two rear-side radars and return both sensor actors."""
    radars = []
    blueprint_library = world.get_blueprint_library()

    for side, y_position, yaw in (
        ("left", -0.85, -135.0),
        ("right", 0.85, 135.0)
    ):
        blueprint = blueprint_library.find("sensor.other.radar")
        blueprint.set_attribute("range", str(BLIND_SPOT_RADAR_RANGE))
        blueprint.set_attribute(
            "horizontal_fov",
            str(BLIND_SPOT_RADAR_HORIZONTAL_FOV)
        )
        blueprint.set_attribute(
            "vertical_fov",
            str(BLIND_SPOT_RADAR_VERTICAL_FOV)
        )
        blueprint.set_attribute(
            "points_per_second",
            str(BLIND_SPOT_RADAR_POINTS_PER_SECOND)
        )
        blueprint.set_attribute(
            "sensor_tick",
            str(BLIND_SPOT_RADAR_SENSOR_TICK)
        )
        transform = carla.Transform(
            carla.Location(x=-0.8, y=y_position, z=1.0),
            carla.Rotation(yaw=yaw)
        )
        radar = world.spawn_actor(
            blueprint,
            transform,
            attach_to=ego_vehicle,
            attachment_type=carla.AttachmentType.Rigid
        )
        radar.listen(
            lambda measurement, radar_side=side: (
                process_blind_spot_radar(measurement, radar_side)
            )
        )
        radars.append(radar)

    print("Left and right blind-spot radars created")
    return radars


# =========================
# Stop sensors
# =========================

def stop_sensors():
    global running

    running = False
