import sys
import numpy as np
import carla

def get_bounding_boxes(fourWheels, twoWheels, walkers, sensor_manager):
        """
        Creates 3D bounding boxes based on carla vehicle list and camera.
        """
        infinite_dst = sensor_manager.cam_id==6

        camera = sensor_manager.sensor
        bounding_boxes_f = [get_bounding_box(vehicle, camera) for vehicle in fourWheels]
        bounding_boxes_t = [get_bounding_box(vehicle, camera) for vehicle in twoWheels]
        bounding_boxes_w = [get_bounding_box(walker, camera) for walker in walkers]
        
        id_fourWheels = np.array([vehicle.id for vehicle in fourWheels])
        id_twoWheels = np.array([vehicle.id for vehicle in twoWheels])
        id_walkers = np.array([walker.id for walker in walkers])
        # filter vehicles not between 0-100m of camera (front)
        
        index_to_delete_f = []
        max_dst = 1000000 if infinite_dst else 110
        for i in range(len(bounding_boxes_f)):
            if  (all(bounding_boxes_f[i][:, 2] < 0) or all(bounding_boxes_f[i][:, 2] >max_dst)):
                index_to_delete_f.append(i)
        bounding_boxes_f = np.delete(bounding_boxes_f, index_to_delete_f, axis=0)
        id_fourWheels = np.delete(id_fourWheels, index_to_delete_f, axis=0)

        # filter tw<o wheels not between 0-80m of camera (front)

        index_to_delete_t = []
        max_dst = 1000000 if infinite_dst else 80
        for i in range(len(bounding_boxes_t)):
            if  (all(bounding_boxes_t[i][:, 2] < 0) or all(bounding_boxes_t[i][:, 2] >max_dst)):
                index_to_delete_t.append(i)
        bounding_boxes_t = np.delete(bounding_boxes_t, index_to_delete_t, axis=0)
        id_twoWheels = np.delete(id_twoWheels, index_to_delete_t, axis=0)

        # filter walkers not between 0-60m of camera (front)
        index_to_delete_w = []
        max_dst = 1000000 if infinite_dst else 50
        for i in range(len(bounding_boxes_w)):
            if  (all(bounding_boxes_w[i][:, 2] < 0) or all(bounding_boxes_w[i][:, 2] >max_dst)):
                index_to_delete_w.append(i)
        bounding_boxes_w = np.delete(bounding_boxes_w, index_to_delete_w, axis=0)
        id_walkers = np.delete(id_walkers, index_to_delete_w, axis=0)

        # print(id_fourWheels.shape, "==", bounding_boxes_f.shape)
        # print(id_twoWheels.shape, "==", bounding_boxes_t.shape)
        # print(id_walkers.shape, "==", bounding_boxes_w.shape)


        #convert to 2d
        bbox_2d_f = get_2d_bbox_from_3d(bounding_boxes_f, id_fourWheels, sensor_manager.w, sensor_manager.h)

        # filter objects hidden by obstacles
        bbox_2d_f = remove_bboxes_not_visible(bbox_2d_f, sensor_manager)

        #convert to 2d
        bbox_2d_t = get_2d_bbox_from_3d(bounding_boxes_t, id_twoWheels, sensor_manager.w, sensor_manager.h)

        # filter objects hidden by obstacles
        bbox_2d_t = remove_bboxes_not_visible(bbox_2d_t, sensor_manager)

        #convert to 2d
        bbox_2d_w = get_2d_bbox_from_3d(bounding_boxes_w, id_walkers, sensor_manager.w, sensor_manager.h)

        # filter objects hidden by obstacles
        bbox_2d_w = remove_bboxes_not_visible(bbox_2d_w, sensor_manager)

        bbox_2d=[]
        for b in bbox_2d_f:
            bbox_2d.append(np.concatenate((b, np.array([2]))))
        for b in bbox_2d_t:
            bbox_2d.append(np.concatenate((b, np.array([1]))))
        for b in bbox_2d_w:
            bbox_2d.append(np.concatenate((b, np.array([0]))))

        # return bbox_2d
        return bbox_2d

def remove_bboxes_not_visible(bounding_boxes, sensor_manager):
    """
    Removes bounding boxes that are not visible.
    """
    index_to_delete = []
    for i in range(len(bounding_boxes)):
        is_visible = is_box_visible(bounding_boxes[i], sensor_manager)
        if not is_visible:
            index_to_delete.append(i)
    
    bounding_boxes = np.delete(bounding_boxes, index_to_delete, axis=0)

    return bounding_boxes

def is_box_visible(box, sensor_manager):

    # check 5 pixel to be sure !
    seg_pixel_1 = sensor_manager.last_seg_img[int((box[2]+box[3])/2), int((box[0]+box[1])/2)]
    seg_pixel_2 = sensor_manager.last_seg_img[int(box[2]+(box[3]-box[2])*2/3), int(box[0]+(box[1]-box[0])*2/3)]
    seg_pixel_3 = sensor_manager.last_seg_img[int(box[2]+(box[3]-box[2])/3), int(box[0]+(box[1]-box[0])/3)]
    seg_pixel_4 = sensor_manager.last_seg_img[int(box[2]+(box[3]-box[2])*2/3), int(box[0]+(box[1]-box[0])/3)]
    seg_pixel_5 = sensor_manager.last_seg_img[int(box[2]+(box[3]-box[2])/3), int(box[0]+(box[1]-box[0])*2/3)]

    visible1 = (seg_pixel_1[0]==60 and seg_pixel_1[1]==20 and seg_pixel_1[2]==220) or (seg_pixel_1[0]==142 and seg_pixel_1[1]==0 and seg_pixel_1[2]==0)
    visible2 = (seg_pixel_2[0]==60 and seg_pixel_2[1]==20 and seg_pixel_2[2]==220) or (seg_pixel_2[0]==142 and seg_pixel_2[1]==0 and seg_pixel_2[2]==0)
    visible3 = (seg_pixel_3[0]==60 and seg_pixel_3[1]==20 and seg_pixel_3[2]==220) or (seg_pixel_3[0]==142 and seg_pixel_3[1]==0 and seg_pixel_3[2]==0)
    visible4 = (seg_pixel_4[0]==60 and seg_pixel_4[1]==20 and seg_pixel_4[2]==220) or (seg_pixel_4[0]==142 and seg_pixel_4[1]==0 and seg_pixel_4[2]==0)
    visible5 = (seg_pixel_5[0]==60 and seg_pixel_5[1]==20 and seg_pixel_5[2]==220) or (seg_pixel_5[0]==142 and seg_pixel_5[1]==0 and seg_pixel_5[2]==0)

    return (visible1 or visible2 or visible3 or visible4 or visible5)

def get_bounding_box(vehicle, camera):
    """
    Returns 3D bounding box for a vehicle based on camera view.
    """

    bb_cords = create_bb_points(vehicle)
    cords_x_y_z = vehicle_to_sensor(bb_cords, vehicle, camera)[:3, :]
    cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
    bbox = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
    camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
    return camera_bbox

def get_2d_bbox_from_3d(bboxes, ids, w, h):
    """
    Transforms 3D bounding boxes to 2D.
    """

    bboxes_2d = []
    for j, bbox in enumerate(bboxes):
        xmin=bbox[0,0]
        xmax=bbox[0,0]
        ymin = bbox[0,1]
        ymax = bbox[0,1]
        for i in range(8):
            if(bbox[i,0]<xmin):
                xmin=bbox[i,0]
            if(bbox[i,0]>xmax):
                xmax=bbox[i,0]
            if(bbox[i,1]<ymin):
                ymin=bbox[i,1]
            if(bbox[i,1]>ymax):
                ymax=bbox[i,1]
        width = xmax-xmin
        height = ymax-ymin

        if(xmin>-width/2 and ymin>-height/2 and xmax<w+width/2 and ymax <h+height/2):
            xmin = max(xmin, 0)
            xmax = min(xmax, w)
            ymin = max(ymin, 0)
            ymax = min(ymax, h)
            bboxes_2d.append([int(xmin),int(xmax),int(ymin),int(ymax), int(ids[j])])

    return bboxes_2d
# def draw_bounding_boxes(display, bounding_boxes):
#         """
#         Draws bounding boxes on pygame display.
#         """

#         bb_surface = pygame.Surface((VIEW_WIDTH, VIEW_HEIGHT))
#         bb_surface.set_colorkey((0, 0, 0))
#         for bbox in bounding_boxes:
#             points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
#             # draw lines
#             # base
#             pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
#             pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
#             pygame.draw.line(bb_surface, BB_COLOR, points[1], points[2])
#             pygame.draw.line(bb_surface, BB_COLOR, points[2], points[3])
#             pygame.draw.line(bb_surface, BB_COLOR, points[3], points[0])
#             # top
#             pygame.draw.line(bb_surface, BB_COLOR, points[4], points[5])
#             pygame.draw.line(bb_surface, BB_COLOR, points[5], points[6])
#             pygame.draw.line(bb_surface, BB_COLOR, points[6], points[7])
#             pygame.draw.line(bb_surface, BB_COLOR, points[7], points[4])
#             # base-top
#             pygame.draw.line(bb_surface, BB_COLOR, points[0], points[4])
#             pygame.draw.line(bb_surface, BB_COLOR, points[1], points[5])
#             pygame.draw.line(bb_surface, BB_COLOR, points[2], points[6])
#             pygame.draw.line(bb_surface, BB_COLOR, points[3], points[7])
#         display.blit(bb_surface, (0, 0))

def create_bb_points(vehicle):
    """
    Returns 3D bounding box for a vehicle.
    """

    cords = np.zeros((8, 4))
    extent = vehicle.bounding_box.extent
    cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
    cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
    cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
    cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
    cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
    cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
    cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
    cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
    return cords

def vehicle_to_sensor(cords, vehicle, sensor):
    """
    Transforms coordinates of a vehicle bounding box to sensor.
    """

    world_cord = vehicle_to_world(cords, vehicle)
    sensor_cord = world_to_sensor(world_cord, sensor)
    return sensor_cord

def vehicle_to_world(cords, vehicle):
    """
    Transforms coordinates of a vehicle bounding box to world.
    """

    bb_transform = carla.Transform(vehicle.bounding_box.location)
    bb_vehicle_matrix = get_matrix(bb_transform)
    vehicle_world_matrix = get_matrix(vehicle.get_transform())
    bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)
    world_cords = np.dot(bb_world_matrix, np.transpose(cords))
    return world_cords

def world_to_sensor(cords, sensor):
    """
    Transforms world coordinates to sensor.
    """

    sensor_world_matrix = get_matrix(sensor.get_transform())
    world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
    sensor_cords = np.dot(world_sensor_matrix, cords)
    return sensor_cords

def get_matrix(transform):
    """
    Creates matrix from carla transform.
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