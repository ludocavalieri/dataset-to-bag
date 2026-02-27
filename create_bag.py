# General imports
import os
import yaml

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

# TODO: Download data
# TODO: Temporarily save to data folder 

# Convert to bag 
if bag_name is not None:
    convert_data_to_bag(save_images, save_imu, save_gt, bag_name)
else:
    convert_data_to_bag(save_images, save_imu, save_gt)