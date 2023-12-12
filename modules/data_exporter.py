from os import makedirs as mkdir
from os.path import join as osp
import cv2
import numpy as np

class DataExporter:

    def __init__(self, dataset_path):
        self.dataset_root = dataset_path
        self.nb_cams = 0
        self.dataset_paths = []
        mkdir(self.dataset_root, exist_ok=True)
        self.users_pos_file = osp(dataset_path, "users_pos.txt")
        # self.videowriters_raw=[]
        # self.videowriters_drawed=[]
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
        f = open(self.users_pos_file, "w")
        f.close()
            

    def add_cam(self, cam_id, sensor_man):
        if(sensor_man.is_veh):
            cam_path = osp(self.dataset_root,"ego","cam"+str(cam_id))
        else:
            cam_path = osp(self.dataset_root,"infra","cam"+str(cam_id))
        
        self.dataset_paths.append(cam_path)
        # self.videowriters_raw.append(cv2.VideoWriter(osp(cam_path, "raw.mp4"), self.fourcc, 30, (sensor_man.w, sensor_man.h)))
        # self.videowriters_drawed.append(cv2.VideoWriter(osp(cam_path, "drawed.mp4"), self.fourcc, 30, (sensor_man.w, sensor_man.h)))
        mkdir(osp(self.dataset_paths[cam_id], "imgs"), exist_ok=True)
        #mkdir(osp(self.dataset_paths[cam_id], "imgs_drawed"), exist_ok=True)
        
        np.savetxt(osp(self.dataset_paths[cam_id], "h_world2pixel.txt"), sensor_man.h_world2pixel, delimiter=",")
        np.savetxt(osp(self.dataset_paths[cam_id], "h_pixel2world.txt"), sensor_man.h_pixel2world, delimiter=",")
        self.nb_cams += 1
        
    def export_data(self, bboxes, imgs, imgs_drawed, img_id):
        
        for i in range (self.nb_cams):

            img_path = osp(self.dataset_paths[i], "imgs", '{0:06d}'.format(img_id)+".jpeg")
            # img_drawed_path = osp(self.dataset_paths[i], "imgs_drawed", '{0:06d}'.format(img_id)+".jpeg")
            # print("\nsave img "+str(img_id)+" at "+img_path+"\n")
            cv2.imwrite(img_path, imgs[i])

            # self.videowriters_raw[i].write(imgs[i])
            # self.videowriters_drawed[i].write(imgs_drawed[i])

            self.write_bbox_to_file(bboxes[i], img_path, osp(self.dataset_paths[i],"bboxes.txt"))
    
    def export_users_pos(self, fourWheels, twoWheels, walkers, img_id):
        file = open(self.users_pos_file, "a")
        file.write(str(img_id))
        for f in fourWheels:
            file.write(" {:.2f}".format(f.get_transform().location.x)+",{:.2f}".format(f.get_transform().location.y)+",{:.2f}".format(f.get_transform().location.z)+",2")
        for t in twoWheels:
            file.write(" {:.2f}".format(t.get_transform().location.x)+",{:.2f}".format(t.get_transform().location.y)+",{:.2f}".format(t.get_transform().location.z)+",1")
        for w in walkers:
            file.write(" {:.2f}".format(w.get_transform().location.x)+",{:.2f}".format(w.get_transform().location.y)+",{:.2f}".format(w.get_transform().location.z)+",0")
        file.write("\n")

    def write_bbox_to_file(self, bboxes, img_path, file_path):
        with open(file_path, "a") as f:
            f.write(img_path)
            for bbox in bboxes:
                # xmin,  ymin, xmax, ymax, id, cls
                # CHANGES : swap xmax and ymin (normalized with other scripts)
                # CHANGES : swap id and cls (normalized with other scripts)
                f.write(" "+str(bbox[0])+","+str(bbox[2])+","+str(bbox[1])+","+str(bbox[3])+","+str(bbox[5])+","+str(bbox[4]))
            f.write("\n")

    def release(self):
        pass
        # for v in self.videowriters_raw:
        #     v.release()
        # for v in self.videowriters_drawed:
        #     v.release()