from pathlib import Path


# =========================
# Project
# =========================

PROJECT_ROOT = Path(__file__).resolve().parent


# =========================
# CARLA connection
# =========================

HOST = "localhost"
PORT = 2000
CLIENT_TIMEOUT = 5.0
MAP_NAME = "Town10HD"


# =========================
# Simulation
# =========================

RUN_DURATION_SECONDS = 1200
NUMBER_OF_TRAFFIC_VEHICLES = 20


# =========================
# Ego vehicle
# =========================

EGO_VEHICLE_BLUEPRINT = "vehicle.tesla.model3"


# =========================
# RGB camera
# =========================

CAMERA_WIDTH = 800
CAMERA_HEIGHT = 600
CAMERA_FOV = 90
CAMERA_SENSOR_TICK = 0.05

CAMERA_LOCATION_X = 1.5
CAMERA_LOCATION_Z = 1.7
CAMERA_PITCH = 0.0


# =========================
# Dataset collection
# =========================

DATASET_DIRECTORY = "dataset"
IMAGES_DIRECTORY = "images"
DRIVING_LOG_FILENAME = "driving_log.csv"

SAVE_EVERY_N_FRAMES = 4
IMAGE_FILE_EXTENSION = ".jpg"
JPEG_QUALITY = 95


# =========================
# Driving mode
# =========================

# Available values:
# "autopilot"
# "ai"
DRIVING_MODE = "ai"
COLLECTING_DATA = False


# =========================
# Steering model
# =========================

STEERING_MODEL_PATH = (
    PROJECT_ROOT
    / "trained_models"
    / "steering_model_v2.pth"
)

MODEL_IMAGE_HEIGHT = 180
MODEL_IMAGE_WIDTH = 320


# =========================
# AI controller
# =========================

STEERING_GAIN = 0.60

MIN_STEERING = -0.35
MAX_STEERING = 0.35

STEERING_SMOOTHING_FACTOR = 0.60

TARGET_SPEED_KMH = 20.0
AI_THROTTLE = 0.22
AI_BRAKE = 0.20

MIN_THROTTLE = 0.0
MAX_THROTTLE = 0.35

MIN_BRAKE = 0.0
MAX_BRAKE = 1.0


# =========================
# Obstacle sensor
# =========================

OBSTACLE_SENSOR_DISTANCE = 20.0
OBSTACLE_SENSOR_HIT_RADIUS = 1.0
OBSTACLE_SENSOR_TICK = 0.05


# =========================
# Safety layer
# =========================

SAFETY_ENABLED = True

# Minimum distances used even at low speed.
SAFETY_MIN_SLOW_DISTANCE = 8.0
SAFETY_MIN_BRAKE_DISTANCE = 5.0
SAFETY_MIN_EMERGENCY_DISTANCE = 2.5

# Dynamic distance contribution according to vehicle speed.
# Distance is calculated as speed_mps * time_gap.
SAFETY_SLOW_TIME_GAP = 1.5
SAFETY_BRAKE_TIME_GAP = 0.9
SAFETY_EMERGENCY_TIME_GAP = 0.4

SAFETY_SLOW_THROTTLE = 0.08
SAFETY_BRAKE_AMOUNT = 0.45
SAFETY_EMERGENCY_BRAKE = 1.0


# =========================
# Controller inactivity
# =========================

CONTROLLER_INACTIVITY_ENABLED = True

# Ignore inactivity while the vehicle is stopped or moving very slowly.
CONTROLLER_INACTIVITY_MIN_SPEED_KMH = 3.0

# Time without a fresh controller response before warning / safe stop.
CONTROLLER_INACTIVITY_WARNING_SECONDS = 2.0
CONTROLLER_INACTIVITY_SAFE_STOP_SECONDS = 5.0

CONTROLLER_INACTIVITY_SAFE_STOP_BRAKE = 1.0


# =========================
# Safety alerts
# =========================

SAFETY_SOUND_ENABLED = True


# =========================
# Lane departure warning
# =========================

LANE_INVASION_ENABLED = True
LANE_INVASION_ALERT_DURATION_SECONDS = 2.0


# =========================
# Lane keeping assist
# =========================

LANE_KEEPING_ENABLED = True
LANE_KEEPING_MIN_SPEED_KMH = 8.0

# Start assisting when either limit is exceeded.
LANE_KEEPING_OFFSET_THRESHOLD_METERS = 0.50
LANE_KEEPING_HEADING_THRESHOLD_DEGREES = 7.0

# Steering correction = -(offset * gain + heading error * gain).
LANE_KEEPING_LATERAL_GAIN = 0.18
LANE_KEEPING_HEADING_GAIN = 0.012
LANE_KEEPING_MAX_CORRECTION = 0.18
LANE_KEEPING_STEERING_LIMIT = 0.45


# =========================
# Traffic light safety
# =========================

TRAFFIC_LIGHT_DETECTION_ENABLED = True
TRAFFIC_LIGHT_RED_BRAKE = 0.75
TRAFFIC_LIGHT_HOLD_BRAKE = 1.0
TRAFFIC_LIGHT_HOLD_SPEED_KMH = 1.0


# =========================
# Route navigation
# =========================

NAVIGATION_ENABLED = True
NAVIGATION_SAMPLING_RESOLUTION_METERS = 2.0
NAVIGATION_MIN_DESTINATION_DISTANCE_METERS = 100.0
NAVIGATION_DESTINATION_REACHED_METERS = 6.0
NAVIGATION_APPROACH_DISTANCE_METERS = 18.0
NAVIGATION_INTERSECTION_LOOKAHEAD_METERS = 7.0
NAVIGATION_RANDOM_SEED = 42
NAVIGATION_AUTO_NEW_ROUTE = True
NAVIGATION_PREFER_JUNCTION_ROUTES = True


# =========================
# Intersection controller
# =========================

INTERSECTION_CONTROLLER_ENABLED = True
INTERSECTION_STEERING_GAIN = 0.018
INTERSECTION_MAX_STEERING = 0.45
INTERSECTION_AI_BLEND = 0.15
INTERSECTION_STEERING_SMOOTHING = 0.45


# =========================
# Road speed controller
# =========================

ROAD_SPEED_CONTROL_ENABLED = True
ROAD_SPEED_DEFAULT_LIMIT_KMH = 30.0
ROAD_SPEED_LIMIT_FACTOR = 0.90
ROAD_SPEED_MAX_TARGET_KMH = 50.0
ROAD_SPEED_JUNCTION_TARGET_KMH = 16.0
ROAD_SPEED_THROTTLE_GAIN = 0.035
ROAD_SPEED_BRAKE_GAIN = 0.055
ROAD_SPEED_MAX_THROTTLE = 0.35
ROAD_SPEED_MAX_BRAKE = 0.60
ROAD_SPEED_DEADBAND_KMH = 1.0
