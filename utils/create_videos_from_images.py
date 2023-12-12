import cv2
import os
from tqdm import tqdm

file_root = "/home/mmorice/Documents/recorded_datas/carla_set_1"

for folder in sorted(os.listdir(file_root)):
    if(os.path.isfile(os.path.join(file_root, folder))):
        continue
    print("processing ", folder)
    folder_path = os.path.join(file_root, folder)
    imgs_path = os.path.join(folder_path, "imgs")
    imgs_drawed_path = os.path.join(folder_path, "imgs_drawed")
    
    firstimg = os.path.join(imgs_path, os.listdir(imgs_path)[0])
    img = cv2.imread(firstimg)

    height, width, channels = img.shape

    out_raw = cv2.VideoWriter(os.path.join(folder_path, "imgs.avi"), cv2.VideoWriter_fourcc(*'MJPG'), 24, (width, height))
    out_drawed = cv2.VideoWriter(os.path.join(folder_path, "imgs_drawed.avi"), cv2.VideoWriter_fourcc(*'MJPG'), 24, (width, height))

    pbar = tqdm(desc="processing images", total=len(os.listdir(imgs_path)))
    for img_name in sorted(os.listdir(imgs_path)):
        img_path = os.path.join(imgs_path, img_name)
        img_drawed_path = os.path.join(imgs_drawed_path, img_name)
        img = cv2.imread(img_path)
        out_raw.write(img)
        img_drawed = cv2.imread(img_drawed_path)
        out_drawed.write(img_drawed)
        pbar.update(1)
    
    out_raw.release()
    out_drawed.release()