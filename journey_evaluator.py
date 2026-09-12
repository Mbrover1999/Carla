import math


class JourneyEvaluator:
    def __init__(self):
        self.previous_location = None
        self.distance_m = 0.0
        self.speed_sum = 0.0
        self.speed_samples = 0
        self.max_speed_kmh = 0.0
        self.efficiency_sum = 0.0
        self.efficiency_samples = 0
        self.collisions = 0
        self.collision_active = False
        self.lane_event_keys = set()
        self.interventions = 0
        self.emergency_interventions = 0
        self.previous_safety_state = "CLEAR"

    def update(
        self,
        location,
        speed_kmh,
        target_speed_kmh,
        safety_state,
        collision=False,
        lane_event_key=None
    ):
        if self.previous_location is not None:
            step_distance = math.hypot(
                location.x - self.previous_location[0],
                location.y - self.previous_location[1]
            )

            # Ignore teleports or map resets.
            if step_distance <= 10.0:
                self.distance_m += step_distance

        self.previous_location = (location.x, location.y)
        self.speed_sum += max(0.0, speed_kmh)
        self.speed_samples += 1
        self.max_speed_kmh = max(self.max_speed_kmh, speed_kmh)

        normalized_state = safety_state or "CLEAR"
        blocked_for_efficiency = any(
            token in normalized_state
            for token in (
                "BRAKING",
                "EMERGENCY",
                "SAFE_STOP",
                "RED_",
                "CREEPING"
            )
        )

        if (
            target_speed_kmh is not None
            and target_speed_kmh > 1.0
            and not blocked_for_efficiency
        ):
            self.efficiency_sum += min(
                max(speed_kmh / target_speed_kmh, 0.0),
                1.0
            )
            self.efficiency_samples += 1

        active = normalized_state not in ("CLEAR", "DISABLED")
        previously_active = self.previous_safety_state not in (
            "CLEAR",
            "DISABLED"
        )

        if active and (
            not previously_active
            or normalized_state != self.previous_safety_state
        ):
            self.interventions += 1

            if "EMERGENCY" in normalized_state:
                self.emergency_interventions += 1

        self.previous_safety_state = normalized_state

        if collision and not self.collision_active:
            self.collisions += 1

        self.collision_active = bool(collision)

        if lane_event_key is not None:
            self.lane_event_keys.add(str(lane_event_key))

    def result(self, duration_seconds, termination_reason):
        average_speed = (
            self.speed_sum / self.speed_samples
            if self.speed_samples
            else 0.0
        )
        efficiency_ratio = (
            self.efficiency_sum / self.efficiency_samples
            if self.efficiency_samples
            else 0.0
        )
        lane_departures = len(self.lane_event_keys)

        safety_score = max(
            0.0,
            50.0
            - self.collisions * 40.0
            - self.emergency_interventions * 3.0
        )
        lane_score = max(0.0, 20.0 - lane_departures * 5.0)
        efficiency_score = 20.0 * efficiency_ratio
        completion_score = (
            10.0
            if termination_reason == "COMPLETED"
            else 5.0
            if termination_reason == "STOPPED"
            else 0.0
        )
        score = round(
            safety_score
            + lane_score
            + efficiency_score
            + completion_score
        )
        score = max(0, min(score, 100))

        if self.collisions:
            rating = "Unsafe"
        elif score >= 90:
            rating = "Excellent"
        elif score >= 75:
            rating = "Good"
        elif score >= 60:
            rating = "Needs improvement"
        else:
            rating = "Poor"

        return {
            "score": score,
            "rating": rating,
            "duration_seconds": round(duration_seconds, 1),
            "distance_km": round(self.distance_m / 1000.0, 3),
            "average_speed_kmh": round(average_speed, 1),
            "max_speed_kmh": round(self.max_speed_kmh, 1),
            "speed_efficiency_percent": round(efficiency_ratio * 100.0, 1),
            "collisions": self.collisions,
            "lane_departures": lane_departures,
            "safety_interventions": self.interventions,
            "emergency_interventions": self.emergency_interventions,
            "termination_reason": termination_reason
        }
