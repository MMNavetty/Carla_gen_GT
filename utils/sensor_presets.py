from modules.sensor_handler import SensorManager

def set_sensor_L_polytech(world, record_manager, videos_width, videos_height):
    cam_id=0
    #2222
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-73.96,Y=-88.10,Z=9.4, Pitch=317,Yaw=233,Roll=0, 
                    attached=None, w=videos_width, h=videos_height,fov=90, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    #2224
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-73.96,Y=-88.10,Z=9.4, Pitch=328,Yaw=107,Roll=0, 
                    attached=None, w=videos_width, h=videos_height,fov=90, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    #2422
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-83.46,Y=34.5,Z=9.4, Pitch=331,Yaw=248,Roll=0,
                    attached=None, w=videos_width, h=videos_height,fov=90, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    #2424
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-25,Y=39,Z=9.4, Pitch=341,Yaw=190,Roll=0.000000,
                    attached=None, w=videos_width, h=videos_height,fov=60, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    #2426
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-56,Y=38,Z=9.4, Pitch=342,Yaw=350,Roll=0.000000,
                    attached=None, w=videos_width, h=videos_height,fov=60, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    #2624
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=97,Y=49,Z=9.4, Pitch=341,Yaw=190,Roll=0.000000,
                    attached=None, w=videos_width, h=videos_height,fov=60, sensor_options={}, cam_id=cam_id))
    cam_id+=1

    
    # ---------------------------- map camera ------------------------------------------
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=34,Y=-15,Z=150, Pitch=270,Yaw=190,Roll=82,
                    attached=None, w=videos_width, h=videos_height,fov=90, sensor_options={}, cam_id=cam_id))
    
    return record_manager

def set_sensor_crossInter_infra(world, record_manager, videos_width, videos_height):
    cam_id=0
    #2222
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-73.96,Y=-88.10,Z=9.4, Pitch=317,Yaw=233,Roll=0, 
                    attached=None, w=videos_width, h=videos_height,fov=90, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    #2224
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-73.96,Y=-88.10,Z=9.4, Pitch=328,Yaw=107,Roll=0, 
                    attached=None, w=videos_width, h=videos_height,fov=90, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    #2422
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-83.46,Y=34.5,Z=9.4, Pitch=331,Yaw=248,Roll=0,
                    attached=None, w=videos_width, h=videos_height,fov=90, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    #2424
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-25,Y=39,Z=9.4, Pitch=341,Yaw=190,Roll=0.000000,
                    attached=None, w=videos_width, h=videos_height,fov=60, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    #2426
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-56,Y=38,Z=9.4, Pitch=342,Yaw=350,Roll=0.000000,
                    attached=None, w=videos_width, h=videos_height,fov=60, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    #2624
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=97,Y=49,Z=9.4, Pitch=341,Yaw=190,Roll=0.000000,
                    attached=None, w=videos_width, h=videos_height,fov=60, sensor_options={}, cam_id=cam_id))
    cam_id+=1

    
    # ---------------------------- map camera ------------------------------------------
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=34,Y=-15,Z=150, Pitch=270,Yaw=190,Roll=82,
                    attached=None, w=videos_width, h=videos_height,fov=90, sensor_options={}, cam_id=cam_id))
    
    return record_manager, cam_id

def set_sensor_crossInter_veh(world, record_manager, vehicle, videos_width, videos_height, cam_id):

    # pitch : up/down 360 : forward, 270 : down, 180 : upsidedown
    # yaw : rotate around scene
    # roll : on side, always 0

    # from unreal values, divide by 100, unreal order : roll, yaw, pitch

    #Lidar
    record_manager.add_sensor(SensorManager(world, 'LiDAR', X=0,Y=0,Z=2.4, Pitch=0,Yaw=0,Roll=0, 
                    attached=vehicle, w=1000, h=1000,fov=90, sensor_options=
                    {'channels' : '64', 'range' : '100',  'points_per_second': '250000', 'rotation_frequency': '20'},
                    cam_id=cam_id, is_veh_attached=True))
    cam_id+=1
    
    # embed cams

    # front 1
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=2.2,Y=-.8,Z=1.1, Pitch=0,Yaw=0,Roll=0,
                    attached=vehicle, w=videos_width, h=videos_height,fov=60, sensor_options={}, cam_id=cam_id, is_veh_attached=True))
    cam_id+=1

    # front 2
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=2.2,Y=.8,Z=1.1, Pitch=0,Yaw=0,Roll=0,
                    attached=vehicle, w=videos_width, h=videos_height,fov=60, sensor_options={}, cam_id=cam_id, is_veh_attached=True))
    cam_id+=1

    #side 1
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=0,Y=-1.5,Z=1.1, Pitch=0,Yaw=270,Roll=0,
                    attached=vehicle, w=videos_width, h=videos_height,fov=90, sensor_options={}, cam_id=cam_id, is_veh_attached=True))
    cam_id+=1
    #side 2
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=0,Y=1.5,Z=1.1, Pitch=0,Yaw=90,Roll=0,
                    attached=vehicle, w=videos_width, h=videos_height,fov=90, sensor_options={}, cam_id=cam_id, is_veh_attached=True))
    cam_id+=1

    # rear
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-3,Y=0,Z=1.1, Pitch=0,Yaw=180,Roll=0,
                    attached=vehicle, w=videos_width, h=videos_height,fov=60, sensor_options={}, cam_id=cam_id, is_veh_attached=True))
    cam_id+=1
    
    return record_manager

def set_sensor_Ariane(world, record_manager, videos_width, videos_height):
    cam_id=0
    # -------------------setup AG--------------------------------
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-7,Y=12,Z=6.5, Pitch=356.398895,Yaw=248.399384,Roll=0.000000, 
                    attached=None, w=videos_width, h=videos_height,fov=60, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-7,Y=12,Z=6.5, Pitch=356.398895,Yaw=187.199554,Roll=0.000000,
                    attached=None, w=videos_width, h=videos_height,fov=60, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-7,Y=12,Z=6.5, Pitch=334.798981,Yaw=316.000000,Roll=0.000000,
                    attached=None, w=videos_width, h=videos_height,fov=80, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-7,Y=12,Z=6.5, Pitch=334.798981,Yaw=36.000000,Roll=0.000000,
                    attached=None, w=videos_width, h=videos_height,fov=80, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-7,Y=12,Z=6.5, Pitch=334.798981,Yaw=116.000000,Roll=0.000000,
                    attached=None, w=videos_width, h=videos_height,fov=80, sensor_options={}, cam_id=cam_id))
    cam_id+=1
    record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-7,Y=12,Z=6.5, Pitch=305.999268,Yaw=205.999725,Roll=0.000000,
                    attached=None, w=videos_width, h=videos_height,fov=110, sensor_options={}, cam_id=cam_id))
    cam_id+=1

# # ------------------setup # AG2---------------------------------

    # record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-7,Y=12,Z=6.5, Pitch=318,Yaw=130,Roll=0, 
    #                 attached=None, w=videos_width, h=videos_height,fov=115, sensor_options={}, cam_id=cam_id))
    # cam_id+=1
    # record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-7,Y=12,Z=6.5, Pitch=318,Yaw=240,Roll=0,
    #                 attached=None, w=videos_width, h=videos_height,fov=115, sensor_options={}, cam_id=cam_id))
    # cam_id+=1
    # record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-7,Y=12,Z=6.5, Pitch=318,Yaw=10,Roll=0,
    #                 attached=None, w=videos_width, h=videos_height,fov=115, sensor_options={}, cam_id=cam_id))
    # cam_id+=1
    
    # # -------

    # record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-7,Y=12,Z=6.5, Pitch=348,Yaw=260,Roll=0.000000,
    #                 attached=None, w=videos_width, h=videos_height,fov=45, sensor_options={}, cam_id=cam_id))
    # cam_id+=1
    # record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-7,Y=12,Z=6.5, Pitch=348,Yaw=20,Roll=0.000000,
    #                 attached=None, w=videos_width, h=videos_height,fov=45, sensor_options={}, cam_id=cam_id))
    # cam_id+=1
    # record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-7,Y=12,Z=6.5, Pitch=348,Yaw=170,Roll=0.000000,
    #                 attached=None, w=videos_width, h=videos_height,fov=40, sensor_options={}, cam_id=cam_id))
    # cam_id+=1

    
    # # ---------------------------- map camera ------------------------------------------
    # record_manager.add_sensor(SensorManager(world, 'RGBCamera', X=-15,Y=13,Z=70, Pitch=270,Yaw=0,Roll=0,
    #                 attached=None, w=videos_width, h=videos_height,fov=110, sensor_options={}, cam_id=cam_id))

#     # ----------------------------------------------------------------------
    return record_manager