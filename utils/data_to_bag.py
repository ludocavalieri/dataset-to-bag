# Import libraries
import os
import cv2
import yaml
import csv
import numpy as np
from datetime import datetime
import re
import shutil

# ROS-related imports
from sensor_msgs.msg import Image, Imu, CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from rosbag2_py import TopicMetadata

# ===========================================================
#                         HELPERS
# ===========================================================
# Convert timestamp to nanoseconds
def timestamp_to_nanoseconds(timestamp_str):
    # Case 1: underscore format
    if "_" in timestamp_str and "-" not in timestamp_str:
        parts = timestamp_str.split("_")

        # If last part is milliseconds (3 digits)
        if len(parts[-1]) == 3:
            parts[-1] = parts[-1] + "000"  # convert ms → µs

        timestamp_str = "_".join(parts)
        dt = datetime.strptime(timestamp_str, "%Y_%m_%d_%H_%M_%S_%f")
        return int(dt.timestamp() * 1e9)

    # Case 2: ISO format
    dt = datetime.fromisoformat(timestamp_str)
    return int(dt.timestamp() * 1e9)

# Extract timestamp from filename
def extract_timestamp_from_filename(filename):
    match = re.search(r'\d{4}_\d{2}_\d{2}_\d{2}_\d{2}_\d{2}_\d{3,6}', filename)

    if not match:
        raise ValueError(f"Cannot extract timestamp from {filename}")

    return match.group(0)

# Format list for YAML creation
def format_list(data):
    return "[ " + ", ".join(f"{float(x):.16e}" for x in data) + " ]"

# Build timestamp dictionary
def build_timestamp_dict(image_list):
        d = {}
        for path in image_list:
            ts = timestamp_to_nanoseconds(
                extract_timestamp_from_filename(os.path.basename(path)))
            d[ts] = path
        return d

# Save camera info
def save_camera_info_yaml(output_path, camera_name, width, height, K, D, R, P, distortion_model):
    # Flatten nested lists
    K = np.array(K).flatten()
    D = np.array(D).flatten()
    R = np.array(R).flatten()
    P = np.array(P).flatten()

    # Manually write yaml
    with open(output_path, "w") as f:
        f.write("%YAML:1.0\n")
        f.write("---\n")

        f.write(f"camera_name: {camera_name}\n")
        f.write(f"image_width: {int(width)}\n")
        f.write(f"image_height: {int(height)}\n")

        f.write("camera_matrix:\n")
        f.write("   rows: 3\n")
        f.write("   cols: 3\n")
        f.write(f"   data: {format_list(K)}\n")

        f.write("distortion_coefficients:\n")
        f.write("   rows: 1\n")
        f.write(f"   cols: {len(D)}\n")
        f.write(f"   data: {format_list(D)}\n")

        f.write(f"distortion_model: {distortion_model}\n")

        f.write("rectification_matrix:\n")
        f.write("   rows: 3\n")
        f.write("   cols: 3\n")
        f.write(f"   data: {format_list(R)}\n")

        f.write("projection_matrix:\n")
        f.write("   rows: 3\n")
        f.write("   cols: 4\n")
        f.write(f"   data: {format_list(P)}\n")

    print(f"[LOG] Saved camera info: {output_path}")

# ===========================================================
#                        BAG CREATOR
# ===========================================================
def convert_data_to_bag(save_images, save_imu, save_gt, compressed=False, rgb=False, bag_name='katwijk_bag'):
    # -------------------------------------------------------
    #                    INITIALIZATION
    # -------------------------------------------------------
    # Define directories
    directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(directory, 'data')
    config_dir = os.path.join(directory, 'config')
    output_dir = os.path.join(directory, 'output')
    processed_data_dir = os.path.join(data_dir, 'processed-data')

    # Output paths
    output_bag = os.path.join(output_dir, bag_name)

    # Get image files
    left_dir = os.path.join(processed_data_dir, 'left-images')
    right_dir = os.path.join(processed_data_dir, 'right-images')
    left_images = sorted([os.path.join(left_dir, f) for f in os.listdir(left_dir) if f.endswith('.png')])
    right_images = sorted([os.path.join(right_dir, f) for f in os.listdir(right_dir) if f.endswith('.png')])

    left_dict = build_timestamp_dict(left_images)
    right_dict = build_timestamp_dict(right_images)

    common_timestamps = sorted(set(left_dict.keys()) & set(right_dict.keys()))

    img_files = [(left_dict[t], right_dict[t], t) for t in common_timestamps]

    # Get IMU data
    imu_data = []
    imu_data_file = os.path.join(processed_data_dir, 'imu.csv')
    with open(imu_data_file, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        next(reader)  # Skip header
        for row in reader:
            timestamp_str = row[0]
            timestamp_ns = timestamp_to_nanoseconds(timestamp_str)

            ax = float(row[1])
            ay = float(row[2])
            az = float(row[3])
            gx = float(row[4])
            gy = float(row[5])
            gz = float(row[6])

            imu_data.append((timestamp_ns, ax, ay, az, gx, gy, gz))

    # Get GT 
    gps_data = []
    gps_data_file = os.path.join(processed_data_dir, 'gps.csv')
    with open(gps_data_file, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        next(reader)  # Skip header
        for row in reader:
            timestamp_str = row[0]
            timestamp_ns = timestamp_to_nanoseconds(timestamp_str)

            x = float(row[8])
            y = float(row[9])
            z = float(row[10])

            gps_data.append((timestamp_ns, x, y, z))

    # Camera parameters 
    camera_params_path = os.path.join(config_dir, 'camera_params.yaml') 
    camera_projections_path = os.path.join(processed_data_dir, 'rectified_camera_projections.yaml') 
    
    with open(camera_params_path, "r") as yamlfile:
        parameters = yaml.load(yamlfile, Loader=yaml.FullLoader)

    with open(camera_projections_path, "r") as yamlfile:
        projections = yaml.load(yamlfile, Loader=yaml.FullLoader)

    width = parameters['w']
    height = parameters['h']

    K_l = parameters['K_l']
    K_r = parameters['K_r']

    P_l = projections['P_l']
    P_r = projections['P_r']

    distortion_model = parameters['dist_model']
    dist_l = np.zeros((1, 4))
    dist_r = np.zeros((1, 4))

    R = np.eye(3)

    # -------------------------------------------------------
    #                    LOG CAMERA INFO
    # -------------------------------------------------------
    # Create dedicated yaml files for camera info
    left_yaml_path = os.path.join(output_dir, "left_camera_info.yaml")
    right_yaml_path = os.path.join(output_dir, "right_camera_info.yaml")

    # Save camera info
    save_camera_info_yaml(left_yaml_path, "katwijk_left", width, height, K_l, dist_l, R, P_l, distortion_model)
    save_camera_info_yaml(right_yaml_path, "katwijk_right", width, height, K_r, dist_r, R, P_r, distortion_model)

    # -------------------------------------------------------
    #                     WRITE ROSBAG
    # -------------------------------------------------------
    # Output path
    output_dir = os.path.join(directory, 'output')
    output_bag = os.path.join(output_dir, bag_name)
    if os.path.exists(output_bag):
        shutil.rmtree(output_bag)

    # Set up ROS2 bag writer
    storage_options = StorageOptions(uri=output_bag, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                        output_serialization_format='cdr')

    writer = SequentialWriter()
    writer.open(storage_options, converter_options)

    # -------------------------------------------------------
    # Images
    # -------------------------------------------------------
    if save_images:
        # Create topics 
        if compressed:
            writer.create_topic(TopicMetadata(
                name='/left/image_rect/compressed',
                type='sensor_msgs/msg/CompressedImage',
                serialization_format='cdr'
            ))
            writer.create_topic(TopicMetadata(
                name='/right/image_rect/compressed',
                type='sensor_msgs/msg/CompressedImage',
                serialization_format='cdr'
            ))
        else: 
            writer.create_topic(TopicMetadata(
                name='/left/image_rect',
                type='sensor_msgs/msg/Image',
                serialization_format='cdr'
            ))
            writer.create_topic(TopicMetadata(
                name='/right/image_rect',
                type='sensor_msgs/msg/Image',
                serialization_format='cdr'
            ))

        # Iterate over image pairs
        for left_path, right_path, timestamp in img_files: 
            # Read images
            left_img = cv2.imread(left_path, cv2.IMREAD_COLOR)
            right_img = cv2.imread(right_path, cv2.IMREAD_COLOR)

            if rgb and not compressed:
                left_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2RGB)
                right_img = cv2.cvtColor(right_img, cv2.COLOR_BGR2RGB)

            # Headers
            left_header = Header(stamp=Time(sec=int(timestamp // 1e9), nanosec=int(timestamp % 1e9)),
                                frame_id='left_camera')
            right_header = Header(stamp=Time(sec=int(timestamp // 1e9), nanosec=int(timestamp % 1e9)),
                                frame_id='right_camera')

            # Image messages
            if compressed:
                # Encode as JPEG 
                _, left_encoded = cv2.imencode('.jpg', left_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                _, right_encoded = cv2.imencode('.jpg', right_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])

                # Create messages
                left_msg = CompressedImage(header=left_header, format='jpeg', data=left_encoded.tobytes())
                right_msg = CompressedImage(header=right_header, format='jpeg', data=right_encoded.tobytes())
            
                # Write to bag
                writer.write('/left/image_rect/compressed', serialize_message(left_msg), timestamp)
                writer.write('/right/image_rect/compressed', serialize_message(right_msg), timestamp)
            else: 
                # Create messages
                encoding = 'rgb8' if rgb else 'bgr8'
                left_msg = Image(header=left_header, height=height, width=width,
                                encoding=encoding, is_bigendian=0, step=width*3, data=left_img.tobytes())
                right_msg = Image(header=right_header, height=height, width=width,
                                encoding=encoding, is_bigendian=0, step=width*3, data=right_img.tobytes())
                
                # Write to bag
                writer.write('/left/image_rect', serialize_message(left_msg), timestamp)
                writer.write('/right/image_rect', serialize_message(right_msg), timestamp)  
               
    # -------------------------------------------------------
    # IMU
    # -------------------------------------------------------
    if save_imu:
        # Create topic
        writer.create_topic(TopicMetadata(
            name='/imu_data',
            type='sensor_msgs/msg/Imu',
            serialization_format='cdr'
        ))

        # Iterate over IMU readings
        for timestamp, ax, ay, az, gx, gy, gz in imu_data:
            # Header 
            imu_header = Header(stamp=Time(sec=int(timestamp // 1e9), nanosec=int(timestamp % 1e9)),
                                frame_id='imu')
            
            # IMU message
            imu_msg = Imu()
            imu_msg.header = imu_header

            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            # Write to bag
            writer.write('/imu_data', serialize_message(imu_msg), timestamp)

    # -------------------------------------------------------
    # Ground Truth
    # -------------------------------------------------------
    if save_gt:
        # Create topic
        writer.create_topic(TopicMetadata(
            name='/ground_truth',
            type='nav_msgs/msg/Odometry',
            serialization_format='cdr'
        ))

        # Iterate over GT readings
        for timestamp, x, y, z in gps_data:
            # Header 
            gt_header = Header(stamp=Time(sec=int(timestamp // 1e9), nanosec=int(timestamp % 1e9)),
                                frame_id='map')
            
            # Ground truth message
            gt_msg = Odometry()
            gt_msg.header = gt_header

            gt_msg.pose.pose.position.x = x
            gt_msg.pose.pose.position.y = y
            gt_msg.pose.pose.position.z = z

            # No orientation available
            gt_msg.pose.pose.orientation.w = 1.0

            # Write to bag
            writer.write('/ground_truth', serialize_message(gt_msg), timestamp)

    # Final message
    print('\033[92m' + f'\n[LOG] Bag created with name {bag_name}.' + '\033[0m')