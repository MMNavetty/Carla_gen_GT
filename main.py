import glob
import os
import sys
import time
import numpy as np
from tqdm import tqdm
import atexit, signal
from modules.args import get_args
from modules.generate_traffic import delete_traffic, gen_traffic, change_actor_behaviour
from modules.sensor_handler import CustomTimer, SensorManager, RecordManager
from utils.sensor_presets import set_sensor_L_polytech
# from utils import weather_list
#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation"""


try:
    path = glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0]
    sys.path.append(path)
    print("loaded "+path)
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import logging
from numpy import random

global client, original_settings, hasExited

hasExited=False

def handle_exit():
    global hasExited

    if hasExited :
        return
    hasExited=True

    try:
        record_manager.data_exporter.release()
    except:
        print("can't release record manager")
        pass
    delete_traffic(client, vehicles_list, walkers_list, all_id, all_actors)
    # try:
    #     delete_traffic(client, vehicles_list, walkers_list, all_id, all_actors)
    # except:
    #     print("can't delete traffic")
    #     pass
    try:
        world.apply_settings(original_settings)
    except:
        print("can't apply original settings")
        pass

atexit.register(handle_exit)
signal.signal(signal.SIGTERM, handle_exit)
signal.signal(signal.SIGINT, handle_exit)

vehicles_list, walkers_list, all_id, all_actors = [],[], [],[]

def run_simulation(args):

    global world, client, record_manager, original_settings
    """This function performed one test run using the args parameters
    and connecting to the carla client passed.
    """
    
     # connect to world
    print("\nconnecting to Carla..\n")
    client = carla.Client(args.host, args.port)
    client.set_timeout(180.0)
    print("connected")
    random.seed(args.seed if args.seed is not None else int(time.time()))
    

    world = client.get_world()
    print("world created")
    traffic_manager = client.get_trafficmanager(args.tm_port)
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)
    if args.respawn:
        traffic_manager.set_respawn_dormant_vehicles(True)
    if args.hybrid:
        traffic_manager.set_hybrid_physics_mode(True)
        traffic_manager.set_hybrid_physics_radius(70.0)
    if args.seed is not None:
        traffic_manager.set_random_device_seed(args.seed)

    print("traffic manager init")
    settings = world.get_settings()
    original_settings = world.get_settings()
    traffic_manager.set_synchronous_mode(True)
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.1

    weather = carla.WeatherParameters(
    cloudiness=0.3,
    precipitation=0.0,
    sun_altitude_angle=110.0,
    sun_azimuth_angle = 90.0,
    precipitation_deposits = 0.0,
    wind_intensity = 0.0,
    fog_density = 0.0,
    wetness = 0.0, 
    )
    world.set_weather(weather)

    print("weather set")
    if args.no_rendering:
        settings.no_rendering_mode = True
    
    world.apply_settings(settings)
    print("primary settings applied to world.")
    record_manager = RecordManager(folder_path = "/home/mmorice/Documents/recorded_datas/"+args.dataset_name, save_datas=True)

    # Then, SensorManager can be used to spawn RGBCamera, LiDARs and SemanticLiDARs as needed
    # and assign each of them to a grid position, 

    videos_width = 1920
    videos_height = 1080
    
    record_manager = set_sensor_L_polytech(world, record_manager, videos_width, videos_height)
    
    print("record manager init.")
    vehicles_list, walkers_list, all_id, all_actors = gen_traffic(args, client, world, traffic_manager)

    print("traffic generated")
    timer = CustomTimer()
    vehicles = world.get_actors().filter('vehicle.*')

    fourWheels = [x for x in vehicles if int(x.attributes['number_of_wheels']) != 2]
    twoWheels = [x for x in vehicles if int(x.attributes['number_of_wheels']) == 2]

    walkers = world.get_actors().filter('walker.*')
    #Simulation loop
    call_exit = False
    previous_frame_time = timer.time()
    print("ending init..")
    for i in range(50):
        if(i%10==0):print("tick "+ str(i) + "/50")
        world.tick()
    print("starting record")
    t=0
    nb_for_changing_behaviour=5
    pbar = tqdm(total = args.nb_img, desc =  "Simulating")
    while t<args.nb_img:
        # Carla Tick
        world.tick()
        if(t%nb_for_changing_behaviour==0):change_actor_behaviour(world, traffic_manager, vehicles_list)
        pbar.update(1)
        # Render received data
        record_manager.record(fourWheels, twoWheels, walkers, t)

        t+=1



def main():
    global hasExited

    argparser = get_args()
    args = argparser.parse_args()

    hasExited=False
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    run_simulation(args)
        

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
