# Katwijck dataset to Rosbag

This repo provides tools and utilities to convert data from the Katwijck beach dataset to a rosbag database compatible with ROS2 Humble.

**Prerequisites**
- [ ] `rosbag2_py` installed ()

## Record bag

First, select the dataset to convert and the topics to record through `bag_cfg.yaml`. Then, access the `dataset-to-bag` folder from terminal and execute `create_bag.py`:

~~~bash
python3 create_bag.py
~~~

This will populate the `output` folder with the following files:

- ($BAG_NAME)/
- `left_camera_info.yaml`
- `right_camera_info.yaml`