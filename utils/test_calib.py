import numpy as np
import cv2

def test_calibration() :
    h = np.loadtxt("/home/mmorice/Documents/recorded_datas/tmp/cam4/homography.txt", delimiter=",")
    # h2 = np.linalg.inv(h)
    # get matrix : from cam sensor to world space
    # inversed : from world space to cam sensor

    ped_path_top_right = np.array([275,139, 1, 1])

    point = np.array([-15, 33, 1, 1.0], dtype=np.float32).reshape((4,1))
    proj = np.dot(h, point)
    proj = proj / proj[2]
    print(proj)
    # proj = (h @ point).flatten()
    # print(h.shape)
    # print(proj/(proj[2]-.5))

    point2 = np.array([275,139, 1])
    invH = np.linalg.pinv(h)
    print(np.dot(invH,h))
    proj2 = (invH @ point2).flatten()
    print(proj2/proj2[2])

    # pts_2d = np.dot(h, ped_path_top_right)

    # pts_2d[0] = pts_2d[0] / pts_2d[2] / -3.17069652e+02 * 33
    # pts_2d[1] = pts_2d[1] / pts_2d[2] / -3.17069652e+02 * 33

    # print(pts_2d)

    # # print(np.dot(h2, ped_path_top_right))
    # world_coord =[-15,33,1]



def brute_force_inversion(width = 600, height = 400) :
    h = np.loadtxt("/home/mmorice/Documents/recorded_datas/tmp/cam4/homography.txt", delimiter=",")
    proj_pts = []
    world_pts = []
    while len(proj_pts) < 8:
        x = np.random.randint(-50, 50)
        y = np.random.randint(-50, 50)
        world_pt = np.array([x, y, 1, 1])
        proj = np.dot(h, world_pt)
        proj = proj / proj[2]
        if(proj[0]>0 and proj[0]<width and proj[1]>0 and proj[1]<height):
            proj_pts.append(proj)
            world_pts.append(world_pt[:3])
    
    invH = cv2.findHomography(np.array(proj_pts), np.array(world_pts))
    print(invH)
    print(invH[0].shape)
    pixel_pt = np.array([275,139,1])
    world_pt_ped = (invH[0] @ pixel_pt).flatten()

    # don't forget to divide by z !!
    world_pt_ped = world_pt_ped / world_pt_ped[2]
    



# test_calibration()
brute_force_inversion()