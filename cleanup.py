import time

import cv2

import sensors


def destroy_existing_vehicles(world):
    vehicles = world.get_actors().filter(
        "vehicle.*"
    )

    for vehicle in vehicles:
        try:
            vehicle.destroy()
            print(
                "Destroyed existing vehicle:",
                vehicle.type_id
            )
        except RuntimeError:
            print(
                "Vehicle was already destroyed"
            )


def cleanup(sensor_list, vehicles):
    sensors.stop_sensors()

    for sensor in sensor_list:
        if sensor is None:
            continue

        try:
            sensor.stop()
            time.sleep(0.2)

            if sensor.is_alive:
                sensor.destroy()

            print(
                "Sensor destroyed:",
                sensor.type_id
            )

        except RuntimeError as error:
            print(
                f"Could not destroy sensor: {error}"
            )

    for vehicle in vehicles:
        if vehicle is None:
            continue

        try:
            if vehicle.is_alive:
                vehicle.destroy()

                print(
                    "Vehicle destroyed:",
                    vehicle.type_id
                )

        except RuntimeError as error:
            print(
                f"Could not destroy vehicle: {error}"
            )

    cv2.destroyAllWindows()

    print("Cleanup completed")