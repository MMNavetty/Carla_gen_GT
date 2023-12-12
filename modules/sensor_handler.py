import carla, pygame
import time
import numpy as np
import cv2
from os import makedirs as mkdir
from copy import deepcopy
import glob
import os
import sys

from math import pi, tan

from modules.make_calibration import make_calibration, brute_force_homography_inversion
from modules.data_exporter import DataExporter
from modules.get_bboxes import get_bounding_boxes
#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Script that render multiple sensors in the same pygame window

By default, it renders four cameras, one LiDAR and one Semantic LiDAR.
It can easily be configure for any different number of sensors. 
To do that, check lines 290-308.
"""


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass



class RecordManager:
    def __init__(self, folder_path, save_datas=True):
        # if not save datas, show results
        self.sensor_list = []
        self.folder_path = folder_path
        self.save_datas = save_datas
        if(save_datas):
            self.data_exporter = DataExporter(folder_path)
        
        


    def add_sensor(self, sensor_man):
        self.sensor_list.append(sensor_man)
        sensor_id = len(self.sensor_list)-1
        print("add sensor id ", sensor_id)
        self.data_exporter.add_cam(sensor_id, sensor_man)
        
        

    def get_sensor_list(self):
        return self.sensor_list

    def record(self, fourWheels, twoWheels, walkers, img_id):
        imgs=[]
        bboxes = []
        imgs_drawed = []
        for sensor_manager in self.sensor_list:

            bb = get_bounding_boxes(fourWheels, twoWheels, walkers, sensor_manager)
            img=sensor_manager.last_img
            
            imgs.append(img)
            bboxes.append(bb)
            imgs_drawed.append(self.draw_bbox_2d(bb, img))
        
                

        if(self.save_datas):
            self.data_exporter.export_data(bboxes, imgs, imgs_drawed, img_id)
            self.data_exporter.export_users_pos(fourWheels, twoWheels, walkers, img_id)
        
        else:
            # if not save datas, show results
            
            
            top = np.hstack((imgs_drawed[0], imgs_drawed[1], imgs_drawed[2]))
            bot = np.hstack((imgs_drawed[3], imgs_drawed[4], imgs_drawed[5]))
            

            cv2.imshow("cams", np.vstack((top, bot)))
            cv2.waitKey(1)
    
    def draw_bbox_3d(self, bboxes, img):
        for bbox in bboxes:
                points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
                # draw lines
                # base
                img = cv2.line(img, points[0], points[1], (0, 255, 0), 2)
                img = cv2.line(img, points[1], points[2], (0, 255, 0), 2)
                img = cv2.line(img, points[2], points[3], (0, 255, 0), 2)
                img = cv2.line(img, points[3], points[0], (0, 255, 0), 2)

                img = cv2.line(img, points[4], points[5], (0, 255, 0), 2)
                img = cv2.line(img, points[5], points[6], (0, 255, 0), 2)
                img = cv2.line(img, points[6], points[7], (0, 255, 0), 2)
                img = cv2.line(img, points[7], points[4], (0, 255, 0), 2)

                img = cv2.line(img, points[0], points[4], (0, 255, 0), 2)
                img = cv2.line(img, points[1], points[5], (0, 255, 0), 2)
                img = cv2.line(img, points[2], points[6], (0, 255, 0), 2)
                img = cv2.line(img, points[3], points[7], (0, 255, 0), 2)

        return img
    
    def draw_bbox_2d(self, bboxes_2d, img):
        img_edited = deepcopy(img)
        for bbox in bboxes_2d:
            if(bbox[4]==0):
                color=(255,0,0)
            elif(bbox[4]==1):
                color=(0,255,0)
            elif(bbox[4]==2):
                color=(0,0,255)
            else:
                color=(0,0,0)


            img_edited = cv2.rectangle(img_edited, (bbox[0], bbox[2]), (bbox[1], bbox[3]), color, 2)
        return img_edited

    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()

class SensorManager:
    def __init__(self, world, sensor_type, X, Y, Z, Pitch, Yaw, Roll, attached, w, h, fov, sensor_options, cam_id, is_veh_attached=False):
        
        self.is_veh=is_veh_attached
        self.cam_transform = carla.Transform(carla.Location(x=X, y=Y, z=Z), carla.Rotation(pitch=Pitch, yaw=Yaw, roll=Roll))
        self.surface = None
        self.world = world
        self.sensor, self.sensor_bp = self.init_sensor(sensor_type, self.cam_transform, attached, w, h, fov, sensor_options)
        self.cam2worldMatrix = self.get_matrix(self.cam_transform)
        # v do NOT delete this, used for bbox detection v
        self.seg_sensor= self.init_sensor("semantic_segmentation", self.cam_transform, attached, w, h, fov, {})
        
        self.sensor_options = sensor_options
        self.timer = CustomTimer()
        self.cam_id=cam_id

        self.time_processing = 0.0
        self.tics_processing = 0
        self.last_img=None
        self.last_depth_img=None
        self.last_seg_img=None

        self.w=w
        self.h=h
        self.fov=fov

        calibration = np.identity(3)
        calibration[0, 2] = w / 2.0
        calibration[1, 2] = h / 2.0
        calibration[0, 0] = calibration[1, 1] = w / (2.0 * np.tan(fov * np.pi / 360.0))
        
        # K matrix

        self.sensor.calibration = calibration
        self.h_world2pixel = make_calibration(self)
        if not is_veh_attached:
            self.h_pixel2world = brute_force_homography_inversion(self.h_world2pixel, self.w, self.h)
        else:
            self.h_pixel2world = self.h_world2pixel

        # self.depth_sensor.calibration = calibration

    def init_sensor(self, sensor_type, transform, attached, w, h, fov, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', str(w))
            camera_bp.set_attribute('image_size_y', str(h))
            camera_bp.set_attribute('fov', str(fov))
            camera_bp.set_attribute('enable_postprocess_effects', str(True))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            if(attached != None):
                camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            else:
                camera = self.world.spawn_actor(camera_bp, transform)
            camera.listen(self.render)

            return camera, camera_bp
        
        elif sensor_type == 'DepthCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
            camera_bp.set_attribute('image_size_x', str(w))
            camera_bp.set_attribute('image_size_y', str(h))
            camera_bp.set_attribute('fov', str(fov))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            if(attached != None):
                camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            else:
                camera = self.world.spawn_actor(camera_bp, transform)
            camera.listen(self.depth_render)

            return camera, camera_bp
        
        elif sensor_type == 'semantic_segmentation':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
            camera_bp.set_attribute('image_size_x', str(w))
            camera_bp.set_attribute('image_size_y', str(h))
            camera_bp.set_attribute('fov', str(fov))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            if(attached != None):
                camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            else:
                camera = self.world.spawn_actor(camera_bp, transform)
            camera.listen(self.seg_render)

            return camera, camera_bp

        elif sensor_type == 'LiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('range', '100')
            lidar_bp.set_attribute('dropoff_general_rate', lidar_bp.get_attribute('dropoff_general_rate').recommended_values[0])
            lidar_bp.set_attribute('dropoff_intensity_limit', lidar_bp.get_attribute('dropoff_intensity_limit').recommended_values[0])
            lidar_bp.set_attribute('dropoff_zero_intensity', lidar_bp.get_attribute('dropoff_zero_intensity').recommended_values[0])

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            if(attached != None):
                print(type(lidar_bp))
                print(type(transform))
                print(type(attached))
                lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)
            else:
                lidar = self.world.spawn_actor(lidar_bp, transform)

            lidar.listen(self.save_lidar_image)

            return lidar, lidar_bp
        
        elif sensor_type == 'SemanticLiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            lidar_bp.set_attribute('range', '100')

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_semanticlidar_image)

            return lidar, lidar_bp
        
        elif sensor_type == "Radar":
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            for key in sensor_options:
                radar_bp.set_attribute(key, sensor_options[key])

            radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
            radar.listen(self.save_radar_image)

            return radar, radar_bp
        
        else:
            print("can't find sensor type. return")
            return None, None
        
    def writeCameraMatrix(self, path):
        camera_bp = self.camera_bp
        cameraFOV = camera_bp.get_attribute('fov').as_float()
        cameraFocal = camera_bp.get_attribute('focal_distance').as_float()
        # print(cameraFocal)
        img_w = camera_bp.get_attribute('image_size_x').as_int()
        img_h = camera_bp.get_attribute('image_size_y').as_int()
        focal = img_w / (2 * tan(cameraFOV * pi / 360))
        c_u = img_w / 2.0
        c_v = img_h / 2.0
        k = np.zeros((3,3), dtype=float)
        k[0,0] = focal 
        k[1,1] = focal
        k[2,2] = 1.0
        k[0,2] = c_u
        k[1,2] = c_v
        # print('Camera Matrix:')
        # print(k)
        if not os.path.exists(path):
            try:
                os.makedirs(path)
            except OSError:
                print('Failed creating directory %s' % path)
        if not os.path.exists('%s/cameraMatrix.npy'%path):
            np.save('%s/cameraMatrix.npy'%path, k)

    def get_sensor(self):
        return self.sensor

    def render(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.last_img= cv2.cvtColor(array, cv2.COLOR_BGR2RGB)


        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1
    
    def seg_render(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.CityScapesPalette)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.last_seg_img= cv2.cvtColor(array, cv2.COLOR_BGR2RGB)


        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def depth_render(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        # print(array[0,20])
        
        
        
        self.last_depth_img= array

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def destroy(self):
        self.sensor.destroy()

    
    def save_lidar_image(self, image):
        t_start = self.timer.time()
        disp_size = [self.w, self.h]
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)

        self.last_img = np.zeros((lidar_img_size), dtype=np.uint8)

        self.last_img[tuple(lidar_data.T)] = (255, 255, 255)

        # if self.display_man.render_enabled():
        #     self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    @staticmethod
    def get_matrix(transform):
        """
        Creates matrix from carla transform ( cam2world)
        np.linalg.inv -> space2cam
        """

        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix
