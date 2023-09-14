import carla
import random
import time

# Connect to the CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# Load a map and create a world
world = client.load_world('Town01')

# Get the map and define a pedestrian crossing location
town_map = world.get_map()
crosswalk_location = carla.Location(x=70, y=50, z=0.1)  # Adjust the location as needed

# Create a pedestrian blueprint
pedestrian_bp = random.choice(world.get_blueprint_library().filter('walker.pedestrian.*'))

# Define pedestrian attributes
pedestrian_attributes = carla.WalkerControl()
pedestrian_attributes.speed = 1.0  # Set pedestrian walking speed (m/s)

# Spawn a pedestrian at the crosswalk
pedestrian = world.spawn_actor(pedestrian_bp, crosswalk_location)
pedestrian_control = carla.WalkerControl()
pedestrian_control.speed = 1.0
pedestrian.set_simulate_physics(True)

try:
    while True:
        # Generate a random number to determine if the pedestrian crosses the road
        crossing_probability = random.uniform(0, 1)
        
        # Adjust the crossing probability threshold as needed
        if crossing_probability < 0.1:  # Adjust this threshold
            pedestrian_control.speed = 0.0  # Stop at the crosswalk
        else:
            pedestrian_control.speed = 1.0  # Continue walking
        
        # Apply pedestrian control
        pedestrian.apply_control(pedestrian_control)
        
        time.sleep(1)  # Adjust the simulation time step as needed

finally:
    # Cleanup when done
    pedestrian.destroy()