import random
import time
import numpy as np
import cv2
import carla


latest_frame = None
running = True

def process_image(image):
    global latest_frame

    if not running:
        return

    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))

    latest_frame = array[:, :, :3].copy()

client = carla.Client('localhost', 2000)
client.set_timeout(5.0)

world = client.get_world()

for vehicle in world.get_actors().filter("vehicle.*"):
    if  vehicle is not None:
        try:
            vehicle.destroy()
            print("Vehicle destroyed")
        except RuntimeError:
            print("Already destroyed")

if world.get_map().name is not None:
    print("Connected!")
blueprints = world.get_blueprint_library()
vehicle_blueprints = world.get_blueprint_library().filter("vehicle")


spawn_points = world.get_map().get_spawn_points()
vehicle_list = []
ego_vehicle = None

while ego_vehicle is None:
    ego_vehicle = world.try_spawn_actor(
        blueprints.find("vehicle.tesla.model3"),
        random.choice(spawn_points)
    )

ego_vehicle.set_autopilot(True)
print("Ego vehicle spawned:", ego_vehicle.type_id)

for i in range(0, 10):
    vehicle = world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))
    if vehicle is None:
        print("Error creating a vehicle?")
    else:
        print("Vehicle spawned: ", vehicle.type_id)
        vehicle.set_autopilot(True)
        vehicle_list.append(vehicle)



# Create a transform to place the camera on top of the vehicle
camera_init_trans = carla.Transform(carla.Location(x=1.5, z=1.7), carla.Rotation(pitch=-5))

# We create the camera through a blueprint that defines its properties
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_bp.set_attribute("image_size_x", "800")
camera_bp.set_attribute("image_size_y", "600")
camera_bp.set_attribute("fov", "90")
camera_bp.set_attribute("sensor_tick", "0.05")

# We spawn the camera and attach it to our ego vehicle
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)

# Start camera with PyGame callback
camera.listen(process_image)

spectator = world.get_spectator()

end_time = time.time() + 30

while time.time() < end_time:
    transform = ego_vehicle.get_transform()

    spectator_transform = carla.Transform(
        transform.transform(carla.Location(x=-8, z=4)),
        carla.Rotation(
            pitch=-15,
            yaw=transform.rotation.yaw,
            roll=0
        )
    )

    spectator.set_transform(spectator_transform)

    if latest_frame is not None:
        cv2.imshow("CARLA Camera", latest_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    world.wait_for_tick()

running = False
camera.stop()
time.sleep(1)
camera.destroy()

for vehicle in world.get_actors().filter("vehicle.*"):
    if  vehicle is not None:
        try:
            vehicle.destroy()
            print("Vehicle destroyed")
        except RuntimeError:
            print("Already destroyed")


cv2.destroyWindow("CARLA Camera")
print("End")
