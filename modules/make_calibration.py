import numpy as np


import numpy as np
from modules.get_bboxes import world_to_sensor
import cv2

def make_calibration(sensor_manager):

    K = sensor_manager.sensor.calibration

    # P = K * F * [R t]
    flip = np.array([[ 0, 1, 0 ], [ 0, 0, -1 ], [ 1, 0, 0 ]], dtype=np.float32)

    x = sensor_manager.cam_transform.location.x
    y = sensor_manager.cam_transform.location.y
    z = sensor_manager.cam_transform.location.z
    pitch = sensor_manager.cam_transform.rotation.pitch
    roll = sensor_manager.cam_transform.rotation.roll
    yaw = sensor_manager.cam_transform.rotation.yaw
    # roll = values['roll']
    # yaw = values['yaw']
    # f = values['f']
    # Cx = values['Cx']
    # Cy = values['Cy']

    # K = np.array([[f, 0, Cx], [0, f, Cy], [0, 0, 1]], dtype=np.float64)

    c_y = np.cos(np.radians(yaw))
    s_y = np.sin(np.radians(yaw))
    c_r = np.cos(np.radians(roll))
    s_r = np.sin(np.radians(roll))
    c_p = np.cos(np.radians(pitch))
    s_p = np.sin(np.radians(pitch))
    matrix = np.identity(4)
    matrix[0, 3] = x
    matrix[1, 3] = y
    matrix[2, 3] = z
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r
    matrix = np.linalg.inv(matrix)
    
    P = K @ flip @ matrix[:3, :]

    return P


def brute_force_homography_inversion(h_world_to_pixel, width = 600, height = 400) :
    proj_pts = []
    world_pts = []
    while len(proj_pts) < 8:
        x = np.random.randint(-200, 200)
        y = np.random.randint(-200, 200)
        world_pt = np.array([x, y, 1, 1])
        proj = np.dot(h_world_to_pixel, world_pt)
        proj = proj / proj[2]
        if(proj[0]>0 and proj[0]<width and proj[1]>0 and proj[1]<height):
            proj_pts.append(proj)
            world_pts.append(world_pt[:3])
    
    invH = cv2.findHomography(np.array(proj_pts), np.array(world_pts))
    return invH[0]





