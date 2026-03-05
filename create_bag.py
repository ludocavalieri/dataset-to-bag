# General imports
import os
import yaml
from tqdm import tqdm
import requests
import tarfile
import shutil

# Custom modules
from utils.data_to_bag import convert_data_to_bag
from utils.dataset_to_data import convert_dataset_to_data

# ===========================================================
#                       INITIALIZATION
# ===========================================================
# Define directory
directory = os.path.dirname(os.path.abspath(__file__))

# Build paths to config files
bag_config_path = os.path.join(directory, 'config', 'bag_cfg.yaml')
camera_params_path = os.path.join(directory, 'config', 'camera_params.yaml') 

# Read config file
with open(bag_config_path, "r") as yamlfile:
        parameters = yaml.load(yamlfile, Loader=yaml.FullLoader)

# Define params
save_images = parameters['save_images']
save_imu = parameters['save_imu']
save_gt = parameters['save_gt']
bag_name = parameters['bag_name'] 
compressed = parameters['compressed']
dataset_url = parameters['dataset_url']
part_nr = parameters['part_nr']
clean_root = parameters['clean_root']

# Build paths to data files
data_folder = os.path.join(directory, "data")
dataset_root = os.path.join(data_folder, f"Part{part_nr}")
tar_path = os.path.join(data_folder, "dataset.tar")

# ==========================================================
#                     DATASET DOWNLOAD
# ==========================================================
# Download dataset unless already downloaded
if os.path.exists(dataset_root):
    print("[LOG] Dataset already extracted. Skipping download.")
else:
    # ------------------------------------------------------
    # DOWNLOAD ONLY IF TAR NOT PRESENT
    # ------------------------------------------------------
    if not os.path.exists(tar_path):
        print("[LOG] Downloading dataset...")
        response = requests.get(dataset_url, stream=True)
        response.raise_for_status()

        # Get total file size from headers
        total_size = int(response.headers.get('content-length', 0))
        block_size = 8192

        with open(tar_path, "wb") as f, tqdm(
                total=total_size,
                unit='B',
                unit_scale=True,
                desc="Downloading",
                ncols=100
        ) as progress_bar:

            for chunk in response.iter_content(chunk_size=block_size):
                if chunk:
                    f.write(chunk)
                    progress_bar.update(len(chunk))

        print("[LOG] Download completed.")
    else:
        print("[LOG] Dataset archive already downloaded. Skipping download.")

    # ------------------------------------------------------
    # EXTRACT TAR FILE
    # ------------------------------------------------------
    print("[LOG] Extracting dataset (.tar)...")

    with tarfile.open(tar_path, "r") as tar:
        tar.extractall(path=data_folder)

    print("[LOG] Extraction completed.")

# ==========================================================
#                    DATA PROCESSING
# ==========================================================
# Populate data folder 
convert_dataset_to_data(dataset_root)

# ==========================================================
#                    BAG CONVERSION   
# ==========================================================
# Remove dataset root (optional)
if clean_root:
    shutil.rmtree(dataset_root)

# Convert to bag 
if bag_name is not None:
    convert_data_to_bag(save_images, save_imu, save_gt, compressed, bag_name)
else:
    convert_data_to_bag(save_images, save_imu, save_gt, compressed)