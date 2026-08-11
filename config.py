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

SAFETY_SLOW_DISTANCE = 12.0
SAFETY_BRAKE_DISTANCE = 6.0
SAFETY_EMERGENCY_DISTANCE = 3.0

SAFETY_SLOW_THROTTLE = 0.08
SAFETY_BRAKE_AMOUNT = 0.45
SAFETY_EMERGENCY_BRAKE = 1.0
