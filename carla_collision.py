import carla
import time

# Connect to the CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# Load a map and create a world
world = client.load_world('Town01')
settings = world.get_settings()
settings.fixed_delta_seconds = 0.1  # Set a fixed time step for the simulation
world.apply_settings(settings)

# Spawn a vehicle
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
spawn_point = carla.Transform(carla.Location(x=50, y=50, z=2), carla.Rotation(yaw=0))
vehicle = world.spawn_actor(vehicle_bp, spawn_point)

# Define a collision callback function
def collision_callback(event):
    other_actor = event.other_actor
    collision_impulse = event.normal_impulse
    print(f"Collision with {other_actor.type_id} | Impulse: {collision_impulse}")

# Subscribe to the collision sensor of the vehicle
collision_sensor_bp = blueprint_library.find('sensor.other.collision')
collision_sensor_transform = carla.Transform(carla.Location(x=0, y=0, z=0), carla.Rotation(pitch=0, yaw=0, roll=0))
collision_sensor = world.spawn_actor(collision_sensor_bp, collision_sensor_transform, attach_to=vehicle)
collision_sensor.listen(collision_callback)

try:
    while True:
        world.tick()  # Update the simulation
        # Your custom logic here

finally:
    # Remove actors and cleanup when done
    collision_sensor.destroy()
    vehicle.destroy()



# https://github.com/carla-simulator/carla/issues/1553



# https://github.com/carla-simulator/carla/issues/4038

import carla
import numpy as np
import tqdm


COLLISION_STATE = None

def _collision_callback(car_id, event):
    global COLLISION_STATE

    COLLISION_STATE[car_id] = True


def test_collision_sensor(
    num_cars: int = 10,
    num_test_rounds: int = 100,
    
    max_timestep: int = 5000,
    dt: float = 0.1
):
    global COLLISION_STATE

    # Create client
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0) # seconds

    for r in range(num_test_rounds):
        print("Test round {}".format(r))

        # Clear collision state
        COLLISION_STATE = np.zeros(num_cars, dtype=np.bool)

        # Load world
        world = client.load_world('Town04')

        # Synchronous tick mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)

        # Create cars with sensors
        car_list = []
        sensor_list = []
        car_name_list = []

        car_blueprints = world.get_blueprint_library().filter("vehicle.*")
        car_spawn_points = world.get_map().get_spawn_points()
        for car_id in range(num_cars):
            # get car blueprint
            car_bp = np.random.choice(car_blueprints)
    
            # get spawn point
            car_pos = np.random.choice(car_spawn_points)
            car_spawn_points.remove(car_pos)

            # spawn car
            car = world.spawn_actor(car_bp, car_pos)
            car_list.append(car)
            car_name_list.append(car_bp.id)

            # create collision sensor
            sensor_blueprint = world.get_blueprint_library().find("sensor.other.collision")
            sensor = world.spawn_actor(sensor_blueprint, carla.Transform(), attach_to=car)
            sensor.listen(lambda event, id=car_id: _collision_callback(id, event))
            sensor_list.append(sensor)

        world.tick()
            
        # Tick
        passed = False
        for step in tqdm.tqdm(range(max_timestep)):
            if COLLISION_STATE.all():
                passed = True
                break

            # apply controls
            [car.apply_control(carla.VehicleControl(throttle=1.0)) for car in car_list]

            # tick
            world.tick()

        if passed:
            print("PASS at {}".format(step))
        else:
            print("FAIL, exceeded {} steps".format(step))
            # print(car_name_list)
            # print(COLLISION_STATE)
            print([car_name_list[i] for i in range(num_cars) if COLLISION_STATE[i] == False])


if __name__ == "__main__":
    test_collision_sensor()