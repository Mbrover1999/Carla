class AutopilotController:
    def __init__(self, client):
        self.activated = False
        self.traffic_manager = client.get_trafficmanager()

    def activate(self, vehicle):
        vehicle.set_autopilot(True)

        self.traffic_manager.auto_lane_change(
            vehicle,
            False
        )

        self.traffic_manager.random_left_lanechange_percentage(
            vehicle,
            0.0
        )

        self.traffic_manager.random_right_lanechange_percentage(
            vehicle,
            0.0
        )

        self.activated = True

        print("CARLA Autopilot controller activated")

    def update(self, vehicle, frame):
        if not self.activated:
            self.activate(vehicle)

        control = vehicle.get_control()

        information = {
            "raw_steering": control.steer,
            "limited_steering": control.steer,
            "applied_steering": control.steer,
            "throttle": control.throttle,
            "brake": control.brake,
            "speed_kmh": None
        }

        return control, information

    def deactivate(self, vehicle):
        vehicle.set_autopilot(False)
        self.activated = False

        print("CARLA Autopilot controller deactivated")