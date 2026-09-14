from pathlib import Path


# =========================
# Project
# =========================

PROJECT_ROOT = Path(__file__).resolve().parent


# =========================
# CARLA connection
# =========================

# Use IPv4 explicitly. On some Windows configurations "localhost" resolves
# to ::1 while the packaged CARLA RPC server listens only on IPv4.
HOST = "127.0.0.1"
PORT = 2000
CLIENT_TIMEOUT = 15.0
MAP_NAME = "Town10HD"


# =========================
# Simulation
# =========================

RUN_DURATION_SECONDS = 3600
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
# Blind-spot radar
# =========================

BLIND_SPOT_DETECTION_ENABLED = True
BLIND_SPOT_RADAR_RANGE = 15.0
BLIND_SPOT_RADAR_HORIZONTAL_FOV = 70.0
BLIND_SPOT_RADAR_VERTICAL_FOV = 14.0
BLIND_SPOT_RADAR_POINTS_PER_SECOND = 1200
BLIND_SPOT_RADAR_SENSOR_TICK = 0.05
BLIND_SPOT_READING_MAX_AGE_SECONDS = 0.25

# Vehicle-centred protection zone (x is forward, y is right).
BLIND_SPOT_REAR_METERS = 10.0
BLIND_SPOT_FORWARD_METERS = 4.0
BLIND_SPOT_MIN_LATERAL_METERS = 1.0
BLIND_SPOT_MAX_LATERAL_METERS = 5.5


# =========================
# Cut-in prediction
# =========================

CUT_IN_DETECTION_ENABLED = True

# Only same-direction vehicles inside this local corridor are considered.
# A vehicle must also have a real lateral velocity toward the ego lane; this
# prevents ordinary traffic in the neighbouring lane from causing braking.
CUT_IN_FORWARD_RANGE_METERS = 25.0
CUT_IN_REAR_RANGE_METERS = 4.0
CUT_IN_TIME_HORIZON_SECONDS = 2.5
CUT_IN_MIN_LATERAL_APPROACH_MPS = 0.40
CUT_IN_CORRIDOR_MARGIN_METERS = 0.45
CUT_IN_MIN_HEADING_ALIGNMENT = 0.60

# Intervention thresholds are based on predicted time until the two vehicle
# envelopes begin to overlap laterally.
CUT_IN_BRAKE_TIME_SECONDS = 1.8
CUT_IN_EMERGENCY_TIME_SECONDS = 1.0
CUT_IN_EMERGENCY_LONGITUDINAL_METERS = 15.0
CUT_IN_INTERVENTION_HOLD_SECONDS = 1.0
CUT_IN_WARNING_THROTTLE = 0.06
CUT_IN_BRAKE_AMOUNT = 0.65
CUT_IN_EMERGENCY_BRAKE = 1.0


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

# Do not lock the vehicle at the normal braking distance when it is already
# almost stationary.  Let it close the remaining gap very slowly and only
# brake again at the low-speed stopping distance.
SAFETY_CREEP_MAX_SPEED_KMH = 2.0
SAFETY_CREEP_DISTANCE = 5.0
SAFETY_CREEP_BRAKE_DISTANCE = 2.8
SAFETY_CREEP_THROTTLE = 0.05


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

# Emergency response after the controller inactivity latch is reached.
EMERGENCY_PULL_OVER_LOOKAHEAD_METERS = 8.0
EMERGENCY_PULL_OVER_TARGET_SPEED_KMH = 12.0
EMERGENCY_PULL_OVER_STEERING_GAIN = 0.018
EMERGENCY_PULL_OVER_MAX_STEERING = 0.35
EMERGENCY_PULL_OVER_LANE_REACHED_METERS = 0.85
EMERGENCY_PULL_OVER_SHOULDER_ENTRY_METERS = 0.65
EMERGENCY_PULL_OVER_LANE_CLEARANCE_METERS = 10.0
EMERGENCY_PULL_OVER_FORWARD_CLEARANCE_METERS = 40.0
EMERGENCY_PULL_OVER_MERGE_CLEARANCE_METERS = 12.0
EMERGENCY_PULL_OVER_REAR_CLEARANCE_METERS = 8.0
EMERGENCY_PULL_OVER_CORRIDOR_HALF_WIDTH_METERS = 2.4
EMERGENCY_PULL_OVER_THROTTLE = 0.22
EMERGENCY_PULL_OVER_BRAKE = 0.45
EMERGENCY_PULL_OVER_STOPPED_SPEED_KMH = 1.0
EMERGENCY_HORN_INTERVAL_SECONDS = 2.1
EMERGENCY_CALL_DELAY_SECONDS = 120.0


# =========================
# Safety alerts
# =========================

SAFETY_SOUND_ENABLED = True
SAFETY_WARNING_SOUND_PATH = (
    PROJECT_ROOT / "assets" / "sounds" / "adas_warning.wav"
)
SAFETY_URGENT_SOUND_PATH = (
    PROJECT_ROOT / "assets" / "sounds" / "adas_urgent.wav"
)
EMERGENCY_HORN_SOUND_PATH = (
    PROJECT_ROOT / "assets" / "sounds" / "driver_wakeup_alarm.wav"
)
EMERGENCY_CALL_SOUND_PATH = (
    PROJECT_ROOT / "assets" / "sounds" / "simulated_call.wav"
)


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
TRAFFIC_LIGHT_STOP_DISTANCE_METERS = 2.5
TRAFFIC_LIGHT_MIN_BRAKING_RANGE_METERS = 4.0
TRAFFIC_LIGHT_BRAKING_TIME_SECONDS = 0.9
TRAFFIC_LIGHT_CREEP_MAX_SPEED_KMH = 2.0
TRAFFIC_LIGHT_CREEP_THROTTLE = 0.05


# =========================
# Stop-sign safety
# =========================

STOP_SIGN_DETECTION_ENABLED = True
STOP_SIGN_LANDMARK_TYPE = "206"
STOP_SIGN_DETECTION_RANGE_METERS = 35.0
STOP_SIGN_APPROACH_RANGE_METERS = 18.0
STOP_SIGN_LINE_MARGIN_METERS = 1.0
STOP_SIGN_STOP_TOLERANCE_METERS = 0.8
STOP_SIGN_HOLD_SECONDS = 2.0
STOP_SIGN_HOLD_SPEED_KMH = 1.0
STOP_SIGN_CREEP_MAX_SPEED_KMH = 2.0
STOP_SIGN_CREEP_THROTTLE = 0.05
STOP_SIGN_APPROACH_THROTTLE = 0.14
STOP_SIGN_MIN_BRAKING_RANGE_METERS = 7.0
STOP_SIGN_REACTION_TIME_SECONDS = 0.6
STOP_SIGN_ASSUMED_DECELERATION_MPS2 = 4.5
STOP_SIGN_BRAKE_AMOUNT = 0.65
STOP_SIGN_HOLD_BRAKE = 1.0
STOP_SIGN_RELEASE_CLEARANCE_METERS = 5.0


# =========================
# Crossing traffic safety
# =========================

CROSS_TRAFFIC_DETECTION_ENABLED = True
CROSS_TRAFFIC_DETECTION_RADIUS_METERS = 30.0
CROSS_TRAFFIC_PATH_LOOKAHEAD_METERS = 20.0
CROSS_TRAFFIC_TIME_HORIZON_SECONDS = 5.0
CROSS_TRAFFIC_WARNING_TIME_GAP_SECONDS = 2.5
CROSS_TRAFFIC_BRAKE_TIME_GAP_SECONDS = 1.2
CROSS_TRAFFIC_MIN_ACTOR_SPEED_MPS = 1.0
CROSS_TRAFFIC_MIN_BRAKING_ACTOR_SPEED_MPS = 2.5
CROSS_TRAFFIC_WAITING_MAX_SPEED_MPS = 3.0
CROSS_TRAFFIC_WAITING_BRAKE_THRESHOLD = 0.15
CROSS_TRAFFIC_ASSUMED_EGO_SPEED_MPS = 2.5
CROSS_TRAFFIC_MIN_CROSSING_ANGLE_DEGREES = 35.0
CROSS_TRAFFIC_BRAKE = 0.75
CROSS_TRAFFIC_HOLD_BRAKE = 1.0
CROSS_TRAFFIC_HOLD_SPEED_KMH = 1.0


# =========================
# Route navigation
# =========================

NAVIGATION_ENABLED = True
NAVIGATION_SAMPLING_RESOLUTION_METERS = 2.0
NAVIGATION_MIN_DESTINATION_DISTANCE_METERS = 100.0
NAVIGATION_DESTINATION_REACHED_METERS = 6.0
NAVIGATION_APPROACH_DISTANCE_METERS = 18.0
NAVIGATION_INTERSECTION_LOOKAHEAD_METERS = 5.0
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
# Keep a small legal margin while following the posted limit on open roads.
ROAD_SPEED_LIMIT_FACTOR = 0.98
ROAD_SPEED_MAX_TARGET_KMH = 150.0
ROAD_SPEED_JUNCTION_LIMIT_FACTOR = 0.75
ROAD_SPEED_JUNCTION_MAX_TARGET_KMH = 30.0
ROAD_SPEED_THROTTLE_GAIN = 0.045
ROAD_SPEED_BRAKE_GAIN = 0.055
ROAD_SPEED_MAX_THROTTLE = 0.45
ROAD_SPEED_MAX_BRAKE = 0.60
ROAD_SPEED_DEADBAND_KMH = 0.5
