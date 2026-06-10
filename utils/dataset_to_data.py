# Import libraries 
import os
import yaml
import numpy as np
import cv2
import pandas as pd

# ===========================================================
#                         HELPERS
# ===========================================================
# Convert LLH to ECEF
def llh_to_ecef(lat, lon, h):
    a = 6378137.0
    e2 = 6.69437999014e-3

    lat = np.radians(lat)
    lon = np.radians(lon)

    N = a / np.sqrt(1 - e2 * np.sin(lat)**2)

    X = (N + h) * np.cos(lat) * np.cos(lon)
    Y = (N + h) * np.cos(lat) * np.sin(lon)
    Z = (N * (1 - e2) + h) * np.sin(lat)

    return X, Y, Z

# Convert LLH to ENU
def llh_to_enu(lat, lon, h, ref_lat, ref_lon, ref_h):
    """Convert LLH to ENU relative to a reference point."""
    # Convert reference to ECEF
    ref_X, ref_Y, ref_Z = llh_to_ecef(ref_lat, ref_lon, ref_h)
    
    # Convert current point to ECEF
    X, Y, Z = llh_to_ecef(lat, lon, h)
    
    # ECEF offset
    dX = X - ref_X
    dY = Y - ref_Y
    dZ = Z - ref_Z
    
    # Rotation matrices
    lat_rad = np.radians(ref_lat)
    lon_rad = np.radians(ref_lon)
    
    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)
    
    # ENU transformation
    E = -sin_lon * dX + cos_lon * dY
    N = -sin_lat * cos_lon * dX - sin_lat * sin_lon * dY + cos_lat * dZ
    U = cos_lat * cos_lon * dX + cos_lat * sin_lon * dY + sin_lat * dZ
    
    return E, N, U

# ==========================================================
#                    DATA PROCESSING
# ==========================================================
def convert_dataset_to_data(dataset_root, rgb=False):
    # -------------------------------------------------------
    #                    INITIALIZATION
    # -------------------------------------------------------
    # Define directories
    directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_folder = os.path.join(directory, 'data')
    config_folder = os.path.join(directory, 'config')

    # Create directory for storing processed data
    processed_data_folder = os.path.join(data_folder, 'processed-data')
    if not os.path.exists(processed_data_folder):
        os.mkdir(processed_data_folder)

    # ----------------------------------------------------------
    # GPS AND IMU DATA
    # ----------------------------------------------------------
    # Build paths
    gps_path = os.path.join(dataset_root, "gps-utm31.txt")
    imu_path = os.path.join(dataset_root, "imu.txt")

    # Build dataframes
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

    # Update GPS dataframe to X Y Z (ENU)
    ref_lat = gps_df.loc[0, 'latitude']
    ref_lon = gps_df.loc[0, 'longitude']
    ref_h   = gps_df.loc[0, 'altitude']

    E_list = []
    N_list = []
    U_list = []

    for _, row in gps_df.iterrows():
        lat = row['latitude']
        lon = row['longitude']
        h   = row['altitude']

        E, N, U = llh_to_enu(lat, lon, h, ref_lat, ref_lon, ref_h)

        E_list.append(E)
        N_list.append(N)
        U_list.append(U)

    gps_df['E'] = E_list
    gps_df['N'] = N_list
    gps_df['U'] = U_list

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
    gps_df.to_csv(os.path.join(processed_data_folder, "gps.csv"), index=False)
    imu_df.to_csv(os.path.join(processed_data_folder, "imu.csv"), index=False)

    print("\n[LOG] GPS and IMU CSV files created.")

    # ----------------------------------------------------------
    # GPS AND IMU DATA
    # ----------------------------------------------------------
    # Create image folders
    left_out = os.path.join(processed_data_folder, "left-images")
    right_out = os.path.join(processed_data_folder, "right-images")
    os.makedirs(left_out, exist_ok=True)
    os.makedirs(right_out, exist_ok=True)

    # Load camera parameters
    camera_params_path = os.path.join(config_folder, 'camera_params.yaml') 
    with open(camera_params_path, "r") as yamlfile:
        cam_params = yaml.load(yamlfile, Loader=yaml.FullLoader)

    # Read camera params
    K_l = np.array(cam_params["K_l"])
    dist_l = np.array(cam_params["dist_l"])
    K_r = np.array(cam_params["K_r"])
    dist_r = np.array(cam_params["dist_r"])
    R = np.array(cam_params["R"])
    T = np.array(cam_params["t"])

    # Access images in the LocCam folder
    image_folder = os.path.join(dataset_root, "LocCam")
    all_images = os.listdir(image_folder)

    left_files = sorted([f for f in all_images if f.endswith("0.png")])
    right_files = sorted([f for f in all_images if f.endswith("1.png")])

    if len(left_files) == 0:
        raise RuntimeError("[ERROR] No stereo images found.")

    # Get image size
    sample = cv2.imread(os.path.join(image_folder, left_files[0]), cv2.IMREAD_GRAYSCALE)
    h, w = sample.shape
    image_size = (w, h)

    # Compute rectification maps 
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K_l, dist_l,
        K_r, dist_r,
        image_size,
        R, T,
        alpha=0
    )

    # Apply rectification maps # TODO: Save P1 and P2 for later use 
    left_map1, left_map2 = cv2.initUndistortRectifyMap(
        K_l, dist_l, R1, P1, image_size, cv2.CV_16SC2
    )
    right_map1, right_map2 = cv2.initUndistortRectifyMap(
        K_r, dist_r, R2, P2, image_size, cv2.CV_16SC2
    )

    # Save projection matrices
    P_l = P1.copy()
    P_r = P2.copy()
    P_r[0, 3] *= 1e-03
    rectified_params = {
        "P_l": P_l.flatten().tolist(),
        "P_r": P_r.flatten().tolist(),
    }

    camera_proj_path = os.path.join(processed_data_folder, 'rectified_camera_projections.yaml')

    with open(camera_proj_path, "w") as f:
        yaml.dump(rectified_params, f)

    print("[LOG] Saved rectified camera parameters.")

    # Process images
    for lf, rf in zip(left_files, right_files):

        read_flag = cv2.IMREAD_COLOR if rgb else cv2.IMREAD_GRAYSCALE
        img_l = cv2.imread(os.path.join(image_folder, lf), read_flag)
        img_r = cv2.imread(os.path.join(image_folder, rf), read_flag)

        rect_l = cv2.remap(img_l, left_map1, left_map2, cv2.INTER_LINEAR)
        rect_r = cv2.remap(img_r, right_map1, right_map2, cv2.INTER_LINEAR)

        # Save
        cv2.imwrite(os.path.join(left_out, lf), rect_l)
        cv2.imwrite(os.path.join(right_out, rf), rect_r)

    print("[LOG] Saved rectified images.")