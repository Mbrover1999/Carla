import math

import carla
import numpy as np

from config import (
    AI_BRAKE,
    AI_THROTTLE,
    MAX_BRAKE,
    MAX_STEERING,
    MAX_THROTTLE,
    MIN_BRAKE,
    STEERING_GAIN,
    MIN_STEERING,
    MIN_THROTTLE,
    STEERING_MODEL_PATH,
    STEERING_SMOOTHING_FACTOR,
    TARGET_SPEED_KMH
)
from inference.steering_predictor import SteeringPredictor


class AIController:
    def __init__(self):
        self.predictor = SteeringPredictor(
            model_path=STEERING_MODEL_PATH
        )

        self.previous_steering = 0.0
        self.last_raw_prediction = 0.0
        self.last_limited_prediction = 0.0

    def activate(self, vehicle):
        vehicle.set_autopilot(False)

        # Clear any old Autopilot command.
        vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                steer=0.0,
                brake=1.0
            )
        )

        print("AI controller activated")

    def update(self, vehicle, frame):
        if frame is None:
            self._apply_safe_stop(vehicle)
            return None

        raw_prediction = (
            self.predictor.predict_from_numpy(frame)
        )

        scaled_prediction = raw_prediction * STEERING_GAIN

        limited_prediction = float(
            np.clip(
                scaled_prediction,
                MIN_STEERING,
                MAX_STEERING
            )
        )

        smoothed_steering = self._smooth_steering(
            limited_prediction
        )

        speed_kmh = self._calculate_speed_kmh(vehicle)

        throttle, brake = self._calculate_speed_control(
            speed_kmh
        )

        control = carla.VehicleControl(
            throttle=float(
                np.clip(
                    throttle,
                    MIN_THROTTLE,
                    MAX_THROTTLE
                )
            ),
            steer=float(
                np.clip(
                    smoothed_steering,
                    MIN_STEERING,
                    MAX_STEERING
                )
            ),
            brake=float(
                np.clip(
                    brake,
                    MIN_BRAKE,
                    MAX_BRAKE
                )
            ),
            hand_brake=False,
            reverse=False,
            manual_gear_shift=False
        )

        vehicle.apply_control(control)

        self.last_raw_prediction = raw_prediction
        self.last_limited_prediction = control.steer

        return {
             "raw_steering": raw_prediction,
             "scaled_steering": scaled_prediction,
             "limited_steering": limited_prediction,
             "applied_steering": control.steer,
             "throttle": control.throttle,
             "brake": control.brake,
             "speed_kmh": speed_kmh
        }

    def deactivate(self, vehicle):
        vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                steer=0.0,
                brake=1.0
            )
        )

        print("AI controller deactivated")

    def _smooth_steering(self, new_steering):
        alpha = STEERING_SMOOTHING_FACTOR

        smoothed_steering = (
            alpha * new_steering
            + (1.0 - alpha) * self.previous_steering
        )

        self.previous_steering = smoothed_steering

        return smoothed_steering

    @staticmethod
    def _calculate_speed_control(speed_kmh):
        if speed_kmh < TARGET_SPEED_KMH:
            return AI_THROTTLE, 0.0

        if speed_kmh > TARGET_SPEED_KMH + 2.0:
            return 0.0, AI_BRAKE

        return 0.0, 0.0

    @staticmethod
    def _calculate_speed_kmh(vehicle):
        velocity = vehicle.get_velocity()

        speed_mps = math.sqrt(
            velocity.x ** 2
            + velocity.y ** 2
            + velocity.z ** 2
        )

        return speed_mps * 3.6

    @staticmethod
    def _apply_safe_stop(vehicle):
        vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                steer=0.0,
                brake=1.0
            )
        )