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


def cleanup(camera, vehicles):
    sensors.stop_sensors()

    if camera is not None:
        try:
            camera.stop()
            time.sleep(0.2)

            if camera.is_alive:
                camera.destroy()

            print("Camera destroyed")

        except RuntimeError as error:
            print(
                f"Could not destroy camera: {error}"
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