
import glob
import os
import sys
import time
import random
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import logging
from numpy import random


behaviour_dict = {
    "dst_lead_veh" : [.5,2.5],
    "minmax_delta_dst_lead_veh" : [-.5,.5],
    "percent_dst_lead_veh_modif_on_tick" : .1,
    "speed_dif" : [80,100], # value to rnd substract 100 : negative value = exceed speed limit, positive : under speed limit
    "minmax_delta_s_dif":5,
    "percent_speed_change": 20,
    "veh_lane_off" : [-1,1],
    "minmax_delta_lane_off":.2,
    "percent_lane_off_change": 20,
}

def gen_traffic(args, client, world, traffic_manager):

    random.seed(args.seed if args.seed is not None else int(time.time()))
    vehicles_list = []
    walkers_list = []
    all_id = []

    blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
    blueprintsWalkers = get_actor_blueprints(world, args.filterw, args.generationw)

    if args.safe:
        #blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
        blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
        blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('t2')]
        blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
        blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]

    blueprints = sorted(blueprints, key=lambda bp: bp.id)

    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)

    if args.number_of_vehicles < number_of_spawn_points:
        random.shuffle(spawn_points)
    elif args.number_of_vehicles > number_of_spawn_points:
        msg = 'requested %d vehicles, but could only find %d spawn points'
        logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
        args.number_of_vehicles = number_of_spawn_points

    # @todo cannot import these directly.
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    # --------------
    # Spawn vehicles
    # --------------
    print("spawn vehicles")
    batch = []

    init=False
    for n, transform in enumerate(spawn_points):
        if n >= args.number_of_vehicles:
            break
        if(init):
            #first vehicle will be used as ego
            blueprint = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
            init=True
        else:
            blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        
        blueprint.set_attribute('role_name', 'autopilot')

        # spawn the cars and set their autopilot and light state all together
        batch.append(SpawnActor(blueprint, transform)
            .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))
    for response in client.apply_batch_sync(batch, True):
        if response.error:
            logging.error(response.error)
        else:
            vehicles_list.append(response.actor_id)
    print("batch sync set.")

    all_vehicle_actors = world.get_actors(vehicles_list)
    # Set automatic vehicle lights update if specified
    if args.car_lights_on:
        for actor in all_vehicle_actors:
            traffic_manager.update_vehicle_lights(actor, True)

    if args.rnd_behaviour:
        #________________________________________________________________________________________________________________________________________________
        print("set random behaviour")
        # set random behaviour
        for actor in all_vehicle_actors:
            speed_dif = random.uniform(behaviour_dict['speed_dif'][0], behaviour_dict['speed_dif'][1])
            # if(random.randint(0,2)==0 and speed_dif!=100): speed_dif-=100

            dst_lead_veh = random.uniform(behaviour_dict['dst_lead_veh'][0], behaviour_dict['dst_lead_veh'][1])
            veh_lane_off = random.uniform(behaviour_dict['veh_lane_off'][0], behaviour_dict['veh_lane_off'][1])
            print("veh with speed percentage %d, offset %.2f, dst to lead veh %.2f. "%(speed_dif, veh_lane_off, dst_lead_veh))
            traffic_manager.vehicle_percentage_speed_difference(actor, speed_dif)
            traffic_manager.distance_to_leading_vehicle(actor, dst_lead_veh)    # tailgating
            traffic_manager.vehicle_lane_offset(actor, veh_lane_off)

            
    # -------------
    # Spawn Walkers
    # -------------
    # some settings
    percentagePedestriansRunning = 0.15      # how many pedestrians will run
    percentagePedestriansCrossing = 0.05     # how many pedestrians will walk through the road
    if args.seedw:
        world.set_pedestrians_seed(args.seedw)
        random.seed(args.seedw)
    # 1. take all the random locations to spawn
    spawn_points = []
    for i in range(args.number_of_walkers):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if (loc != None):
            spawn_point.location = loc
            spawn_points.append(spawn_point)
    # 2. we spawn the walker object
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        # set as not invincible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        # set the max speed
        if walker_bp.has_attribute('speed'):
            if (random.random() > percentagePedestriansRunning):
                # walking
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                # running
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        else:
            print("Walker has no speed")
            walker_speed.append(0.0)
        batch.append(SpawnActor(walker_bp, spawn_point))
    results = client.apply_batch_sync(batch, True)
    walker_speed2 = []
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2
    # 3. we spawn the walker controller
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id
    # 4. we put together the walkers and controllers id to get the objects from their id
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
    all_actors = world.get_actors(all_id)

    # wait for a tick to ensure client receives the last transform of the walkers we have just created
    world.tick()

    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(all_id), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # max speed
        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

    print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

    # # Example of how to use Traffic Manager parameters
    # traffic_manager.global_percentage_speed_difference(30.0)

    # get ref to destroy them afterwards
    return vehicles_list, walkers_list, all_id, all_actors

def delete_traffic(client, vehicles_list, walkers_list, all_id, all_actors):

    print('\ndestroying %d vehicles' % len(vehicles_list))
    client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

    # stop walker controllers (list is [controller, actor, controller, actor ...])
    for i in range(0, len(all_id), 2):
        all_actors[i].stop()

    print('\ndestroying %d walkers' % len(walkers_list))
    client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

    time.sleep(0.5)


def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []
    

def change_actor_behaviour(world, traffic_manager, vehicles_list):
    all_vehicle_actors = world.get_actors(vehicles_list)
    print("set random behaviour")
    # set random behaviour
    for actor in all_vehicle_actors:
        if(behaviour_dict["percent_speed_change"]>random.random()):
            speed_dif = random.uniform(behaviour_dict['speed_dif'][0], behaviour_dict['speed_dif'][1])
            traffic_manager.vehicle_percentage_speed_difference(actor, speed_dif)
        # if(random.randint(0,2)==0 and speed_dif!=100): speed_dif-=100
        if(behaviour_dict["percent_dst_lead_veh_modif_on_tick"]>random.random()):
            dst_lead_veh = random.uniform(behaviour_dict['dst_lead_veh'][0], behaviour_dict['dst_lead_veh'][1])
            traffic_manager.distance_to_leading_vehicle(actor, dst_lead_veh)    # tailgating
        if(behaviour_dict["percent_lane_off_change"]>random.random()):
            veh_lane_off = random.uniform(behaviour_dict['veh_lane_off'][0], behaviour_dict['veh_lane_off'][1])
            traffic_manager.vehicle_lane_offset(actor, veh_lane_off)
        # print("veh with speed percentage %d, offset %.2f, dst to lead veh %.2f. "%(speed_dif, veh_lane_off, dst_lead_veh))