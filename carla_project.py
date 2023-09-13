
import carla 
import random 
import time 
import math 


discretized_roads={'s0':(), 's1': (), 's2': (), 's3':(), 's4': (), 's5': (), 's6':(), 's7':()}
discretized_crosswalk={'p0':(), 'p1': (), 'p2': ()}


current_location={'A': 1, 'H': 0, 'P': 0}

def get_all_policies():

    with open('all_policies.txt') as file : 
        data =eval(file.read())
    return data 

def distance(x, y):
    return math.sqrt((x[0] - y[0])**2 + (x[1] - y[1])**2 + (x[2] - y[2])**2)

def get_current_discretized_state(location):

    closest_point = min(discretized_roads.values(), key=lambda point: distance(point, location))

    key= [key for key, val in discretized_roads.items() if val==closest_point]
    return key[0]
  










actor_list=[]
policies= get_all_policies()


try:
    client=carla.Client('localhost', 2000)
    client.reload_world()
    world=client.get_world()

    #blueprint
    vehicle_bp=world.get_blueprint_library().filter("vehicle.bmw.*")
    walker_bp= world.get_blueprint_library().filter("walker.pedestrian.*")
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')

    current_location_A=discretized_roads['s1']
    start_point = carla.Transform(carla.Location(current_location_A), 
                                  carla.Rotation(pitch=-4.909854, yaw=-89.507202, roll=-0.000610))
    vehicle_A=world.spawn_actor(random.choice(vehicle_bp), start_point )

    actor_list.append(vehicle_A)

    current_location_H=discretized_roads['s0']
    start_point2 = carla.Transform(carla.Location(current_location_H), 
                                  carla.Rotation(pitch=-4.909854, yaw=-89.507202, roll=-0.000610))
    vehicle_H=world.spawn_actor(random.choice(vehicle_bp), start_point )
    actor_list.append(vehicle_H)


    # pedestrian 
    walker=random.choice(walker_bp)
    spawn_point=carla.Transform(carla.Location(x=96.437691, y=37.796257, z=2.816283), carla.Rotation(pitch=3.125385, yaw=-167.939011, roll=-0.000610))
    pedestrian = world.spawn_actor(walker, spawn_point)

    # walker AI controller 
    controller = world.spawn_actor(walker_controller_bp, carla.Transform(), pedestrian)
    world.wait_for_tick()

    # managing pedestrian 
    controller.start()
    controller.go_to_location(carla.Location(discretized_crosswalk['p2']))
    world.wait_for_tick()

    actor_list.append(pedestrian)
    actor_list.append(controller)

    #spectator...
    spectator=world.get_spectator()
    vehicle_transform=vehicle_H.get_transform()
    transform = carla.Transform(vehicle_transform.transform(carla.Location(x=-4, z=2.5)), vehicle_transform.rotation)
    spectator.set_transform(transform)

    #controller of vehicles ... 




   
    
    while True:
        # 0 - stop  and 1 - go 
        policy_A= policies['agent1']
        tuple_key=tuple(current_location.values())
        string_tuple = tuple(str(item) for item in tuple_key)
        current_policy=policy_A[string_tuple]

        if current_policy[2]!=1:
            vehicle_A.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))

        policy_H= policies['agent2']
        tuple_key2=tuple(current_location.values())
        string_tuple2 = tuple(str(item) for item in tuple_key2)
        current_policy_H=policy_H[string_tuple2]

        if current_policy[2]!=1 and current_policy[0]+1 == current_policy[1]:
            vehicle_H.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))

        
        #updating location... 

        current_location_A=vehicle_A.get_transform()
        current_discretized_state_A=get_current_discretized_state(current_location_A)

        current_location_H=vehicle_H.get_transform()
        current_discretized_state_H=get_current_discretized_state(current_location_H)

        current_location_P=pedestrian.get_transform()
        current_discretized_state_P=get_current_discretized_state(current_location_P)



        print("I am here")























finally:
    for actor in actor_list:
        actor.destroy()

