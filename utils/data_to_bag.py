# Import libraries
import os
import cv2
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from rosbag2_py import TopicMetadata

# TODO: Add static tf to bag? 

# ===========================================================
#                        BAG CREATOR
# ===========================================================
# TODO: Make more efficient depending on data to download
def convert_data_to_bag(save_images, save_imu, save_gt, bag_name='katwijck_bag'):
    # -------------------------------------------------------
    #                    INITIALIZATION
    # -------------------------------------------------------
    # Data directory
    directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(directory, 'data')

    # Get image files
    left_dir = os.path.join(data_dir, 'left-images')
    right_dir = os.path.join(data_dir, 'right-images')
    left_images = sorted([os.path.join(left_dir, f) for f in os.listdir(left_dir) if f.endswith('.png')])
    right_images = sorted([os.path.join(right_dir, f) for f in os.listdir(right_dir) if f.endswith('.png')])
    img_files = list(zip(left_images, right_images))  # pair images

    # Get IMU data
    # TODO

    # Get GT 
    # TODO

    # Camera parameters 
    # TODO

    # -------------------------------------------------------
    #                     WRITE ROSBAG
    # -------------------------------------------------------
    # Output path
    output_dir = os.path.join(directory, 'output')
    output_bag = os.path.join(output_dir, bag_name)

    # Set up ROS2 bag writer
    storage_options = StorageOptions(uri=output_bag, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                        output_serialization_format='cdr')

    writer = SequentialWriter()
    writer.open(storage_options, converter_options)

    # Create topics
    writer.create_topic(TopicMetadata(
        name='/left/image',
        type='sensor_msgs/msg/Image',
        serialization_format='cdr'
    ))
    writer.create_topic(TopicMetadata(
        name='/right/image',
        type='sensor_msgs/msg/Image',
        serialization_format='cdr'
    ))
    writer.create_topic(TopicMetadata(
        name='/left/camera_info',
        type='sensor_msgs/msg/CameraInfo',
        serialization_format='cdr'
    ))
    writer.create_topic(TopicMetadata(
        name='/right/camera_info',
        type='sensor_msgs/msg/CameraInfo',
        serialization_format='cdr'
    ))
    writer.create_topic(TopicMetadata(
        name='/imu_data',
        type='sensor_msgs/msg/Imu',
        serialization_format='cdr'
    ))
    writer.create_topic(TopicMetadata(
        name='/ground_truth',
        type='nav_msgs/msg/Odometry',
        serialization_format='cdr'
    ))

    # Initialize timestamp (in nanoseconds)
    timestamp = 0

    # Set DeltaT depending on frequency?
    # TODO

    # Iterate over image pairs
    for left_path, right_path in img_files: # TODO: Generalize so that it is more general?
        timestamp += int(dt * 1e9)  # 0.5s per frame

        # ---------------------------------------------------
        # Images
        # ---------------------------------------------------
        if save_images:
            # Read images
            left_img = cv2.imread(left_path, cv2.IMREAD_COLOR)
            right_img = cv2.imread(right_path, cv2.IMREAD_COLOR)

            # Convert to bytes
            left_bytes = left_img.tobytes()
            right_bytes = right_img.tobytes()

            # Headers
            left_header = Header(stamp=Time(sec=int(timestamp // 1e9), nanosec=int(timestamp % 1e9)),
                                frame_id='left_camera')
            right_header = Header(stamp=Time(sec=int(timestamp // 1e9), nanosec=int(timestamp % 1e9)),
                                frame_id='right_camera')

            # Image messages
            left_msg = Image(header=left_header, height=height, width=width,
                            encoding='bgr8', is_bigendian=0, step=width*3, data=left_bytes)
            right_msg = Image(header=right_header, height=height, width=width,
                            encoding='bgr8', is_bigendian=0, step=width*3, data=right_bytes)

            # CameraInfo messages
            left_info = CameraInfo(header=left_header, height=height, width=width,
                                distortion_model=distortion_model, d=D, k=K, r=R, p=P_left)
            right_info = CameraInfo(header=right_header, height=height, width=width,
                                    distortion_model=distortion_model, d=D, k=K, r=R, p=P_right)

            # Write to bag
            writer.write('/left/image', serialize_message(left_msg), timestamp)
            writer.write('/right/image', serialize_message(right_msg), timestamp)
            writer.write('/left/camera_info', serialize_message(left_info), timestamp)
            writer.write('/right/camera_info', serialize_message(right_info), timestamp)

        # ---------------------------------------------------
        # IMU
        # ---------------------------------------------------
        if save_imu:
            # Header 
            imu_header = Header(stamp=Time(sec=int(timestamp // 1e9), nanosec=int(timestamp % 1e9)),
                                frame_id='imu')
            
            # IMU message
            imu_msg = Imu(...)

        # ---------------------------------------------------
        # Ground Truth
        # ---------------------------------------------------
        if save_gt:
            # Header 
            gt_header = Header(stamp=Time(sec=int(timestamp // 1e9), nanosec=int(timestamp % 1e9)),
                                frame_id='map')
            
            # Ground truth message
            gt_msg = Odometry(...)

    # Final message
    print(f'Bag created with name {bag_name}. The bag contains {N} messages on the following topics:')