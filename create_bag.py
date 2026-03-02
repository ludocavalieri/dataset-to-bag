# General imports
import os
import yaml

import requests
import tarfile
import pandas as pd
import numpy as np
import cv2
import shutil

# Custom modules
from utils.data_to_bag import convert_data_to_bag

# ===========================================================
#                       INITIALIZATION
# ===========================================================
# Define directory
directory = os.path.dirname(os.path.abspath(__file__))

# Build path to files
bag_config_path = os.path.join(directory, 'config', 'bag_cfg.yaml')
camera_params_path = os.path.join(directory, 'config', 'camera_params.yaml') #? Maybe not needed in main

# Read config file
with open(bag_config_path, "r") as yamlfile:
        parameters = yaml.load(yamlfile, Loader=yaml.FullLoader)

# Define params
save_images = parameters['save_images']
save_imu = parameters['save_imu']
save_gt = parameters['save_gt']
bag_name = parameters['bag_name'] 

# ==========================================================
# CREATE DATA FOLDER STRUCTURE
# ==========================================================

data_folder = os.path.join(directory, "data")
os.makedirs(data_folder, exist_ok=True)

dataset_root = os.path.join(data_folder, "Part1")
dataset_url = "https://roboshare.esa.int/index.php/s/abDQEmi8Yjv9fhP/download"
tar_path = os.path.join(data_folder, "dataset.tar")

# ==========================================================
# CHECK IF DATASET ALREADY EXISTS
# ==========================================================

if os.path.exists(dataset_root):
    print("Dataset already extracted. Skipping download.")
else:

    # ------------------------------------------------------
    # DOWNLOAD ONLY IF TAR NOT PRESENT
    # ------------------------------------------------------
    if not os.path.exists(tar_path):

        print("Downloading dataset...")
        response = requests.get(dataset_url, stream=True)
        response.raise_for_status()

        with open(tar_path, "wb") as f:
            for chunk in response.iter_content(chunk_size=8192):
                f.write(chunk)

        print("Download completed.")
    else:
        print("Dataset archive already downloaded. Skipping download.")

    # ------------------------------------------------------
    # EXTRACT TAR FILE
    # ------------------------------------------------------
    print("Extracting dataset (.tar)...")

    with tarfile.open(tar_path, "r") as tar:
        tar.extractall(path=data_folder)

    print("Extraction completed.")

# ==========================================================
# 3) READ IMU + GPS AND EXPORT CSV
# ==========================================================

gps_path = os.path.join(dataset_root, "gps-utm31.txt")
imu_path = os.path.join(dataset_root, "imu.txt")

gps_df = pd.read_csv(
    gps_path,
    sep=' ',
    header=None,
    names=[
        'timestamp', 'status',
        'latitude', 'longitude', 'altitude',
        'std_lat', 'std_lon', 'std_alt'
    ]
)

imu_df = pd.read_csv(
    imu_path,
    sep=' ',
    header=None,
    names=[
        'timestamp',
        'accel_x', 'accel_y', 'accel_z',
        'gyro_x', 'gyro_y', 'gyro_z',
        'mag_x', 'mag_y', 'mag_z'
    ]
)

# Convert timestamps
gps_df['timestamp'] = pd.to_datetime(
    gps_df['timestamp'],
    format='%Y_%m_%d_%H_%M_%S_%f'
)

imu_df['timestamp'] = pd.to_datetime(
    imu_df['timestamp'],
    format='%Y_%m_%d_%H_%M_%S_%f'
)

# Save CSV
gps_df.to_csv(os.path.join(data_folder, "gps.csv"), index=False)
imu_df.to_csv(os.path.join(data_folder, "imu.csv"), index=False)

print("GPS and IMU CSV files created.")

# ==========================================================
# 4) STEREO RECTIFICATION
# ==========================================================
left_out = os.path.join(data_folder, "left_images")
right_out = os.path.join(data_folder, "right_images")
os.makedirs(left_out, exist_ok=True)
os.makedirs(right_out, exist_ok=True)
# Load camera parameters
with open(camera_params_path, "r") as yamlfile:
    cam_params = yaml.load(yamlfile, Loader=yaml.FullLoader)

K_l = np.array(cam_params["K_l"])
dist_l = np.array(cam_params["dist_l"])
K_r = np.array(cam_params["K_r"])
dist_r = np.array(cam_params["dist_r"])
R = np.array(cam_params["R"])
T = np.array(cam_params["t"])

image_folder = os.path.join(dataset_root, "LocCam")
all_images = os.listdir(image_folder)

left_files = sorted([f for f in all_images if f.endswith("0.png")])
right_files = sorted([f for f in all_images if f.endswith("1.png")])

if len(left_files) == 0:
    raise RuntimeError("No stereo images found.")

# Get image size
sample = cv2.imread(os.path.join(image_folder, left_files[0]), 0)
h, w = sample.shape
image_size = (w, h)

# Compute rectification maps ONCE
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    K_l, dist_l,
    K_r, dist_r,
    image_size,
    R, T,
    alpha=0
)

left_map1, left_map2 = cv2.initUndistortRectifyMap(
    K_l, dist_l, R1, P1, image_size, cv2.CV_16SC2
)

right_map1, right_map2 = cv2.initUndistortRectifyMap(
    K_r, dist_r, R2, P2, image_size, cv2.CV_16SC2
)

print("Rectification maps ready.")

# Process images
for lf, rf in zip(left_files, right_files):

    img_l = cv2.imread(os.path.join(image_folder, lf), 0)
    img_r = cv2.imread(os.path.join(image_folder, rf), 0)

    rect_l = cv2.remap(img_l, left_map1, left_map2, cv2.INTER_LINEAR)
    rect_r = cv2.remap(img_r, right_map1, right_map2, cv2.INTER_LINEAR)

    # Save
    cv2.imwrite(os.path.join(left_out, lf), rect_l)
    cv2.imwrite(os.path.join(right_out, rf), rect_r)

print("Stereo rectification completed.")

# ==========================================================
# CLEAN ORIGINAL EXTRACTED FOLDER (OPTIONAL)
# ==========================================================

shutil.rmtree(dataset_root)

print("Final dataset structure created.")
# Convert to bag 
if bag_name is not None:
    convert_data_to_bag(save_images, save_imu, save_gt, bag_name)
else:
    convert_data_to_bag(save_images, save_imu, save_gt)