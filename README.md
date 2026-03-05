[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/python-3.x-blue)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

# Katwijk Dataset to ROS2 Bag Converter

This repository provides tools to convert data from the **Katwijk Beach Dataset** into a **ROS2 bag (rosbag2)** compatible with **ROS2 Humble**.

> [!NOTE]
> The Katwijk Beach Dataset can be downloaded
> [here](https://roboshare.esa.int/datasets/index.php/katwijk-beach-planetary-rover-dataset/).

## Prerequisites

- ROS2 Humble
- **rosbag2_py** installed

## Creating a ROS2 bag

First, configure the bag by editing [`bag_cfg.yaml`](config/bag_cfg.yaml).  
Then navigate to the `dataset-to-bag` directory and execute the conversion script [`create_bag.py`](dataset-to-bag/create_bag.py):

~~~bash
python3 create_bag.py
~~~

This will populate the [output](output/) folder with the following files:

~~~bash
output/
 ├── <BAG_NAME>/               # Output bag directory
 ├── left_camera_info.yaml     # Left camera info
 └── right_camera_info.yaml    # Right camera info
~~~

After generating the bag, it is recommended to verify its integrity using a visualization tool such as **Foxglove** or another ROS-compatible viewer.

## Playing the ROS2 bag

You can replay the generated bag with:

~~~bash
ros2 bag play output/<BAG_NAME>
~~~

## Contributions

Contributions and feedback are welcome!

Please open issues or submit pull requests for improvements or bug fixes.

## Authors

Ludovica Cavalieri* - ludovica.cavalieri@uniroma1.it

Mohamed El Awag* - mohamed.elawag@uniroma1.it

*Sapienza University of Roma, Department of Mechanical and Aerospace Engineering