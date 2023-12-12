from os.path import join as osp
from os import makedirs as mkdir
import numpy as np, cv2


def brute_force_to_cam6(h_world_to_pixel_currentcam, h_world_to_pixel_cam6, width=512,height=388):
    proj_pts = []
    cam6_pts = []
    while len(proj_pts) < 25:
        x = np.random.randint(-200, 200)
        y = np.random.randint(-200, 200)
        world_pt = np.array([x, y, 1, 1])
        proj = np.dot(h_world_to_pixel_currentcam, world_pt)
        proj = proj / proj[2]
        if(proj[0]>0 and proj[0]<width and proj[1]>0 and proj[1]<height):
            proj_pts.append(proj)

            cam6_proj = np.dot(h_world_to_pixel_cam6, world_pt)
            cam6_proj = cam6_proj / cam6_proj[2]
            cam6_pts.append(cam6_proj)
    
    invH = cv2.findHomography(np.array(proj_pts), np.array(cam6_pts))
    return invH[0]


def get_matrix_configs(root_path):
    cam_ids = range(6)
    map_id = 6
    mkdir(osp(root_path, "configs"), exist_ok=True)
    h_world_to_pixel_cam6 = np.loadtxt(osp(root_path, "cam6", "h_world2pixel.txt"), delimiter=",")
    for cam in cam_ids:
        h_world_to_pixel_currentcam = np.loadtxt(osp(root_path, "cam"+str(cam), "h_world2pixel.txt"), delimiter=",")
        matrix = brute_force_to_cam6(h_world_to_pixel_currentcam, h_world_to_pixel_cam6, width=512,height=388)
        np.savetxt(osp(root_path, "configs", str(cam)+".txt"), matrix)
        print("cam ", cam, " done.")



path = "/home/mmorice/Documents/recorded_datas/carla_222426_w40_v30"


get_matrix_configs(path)