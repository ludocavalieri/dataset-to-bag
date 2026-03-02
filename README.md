# Katwijck dataset to Rosbag

This repo provides tools and utilities to convert data from the Katwijck beach dataset to a rosbag database compatible with ROS2 Humble.

**Prerequisites**
- [ ] `rosbag2_py` installed

## Record bag

First, select the dataset to convert and the topics to record through `bag_cfg.yaml`. Then, access the `dataset-to-bag` folder from terminal and execute `create_bag.py`:

~~~bash
python3 create_bag.py
~~~

This will populate the `output` folder with the following files:

- ($BAG_NAME)/
- `left_camera_info.yaml`
- `right_camera_info.yaml`

Once the bag has been created, we recommend checking its integrity through Foxglove or another visualizer. 

## Contributions

Contributions and feedback are welcome!

Please open issues or submit pull requests for improvements or bug fixes.

## Authors

Ludovica Cavalieri - ludovica.cavalieri@uniroma1.it

Mohamed El Awag - mohamed.elawag@uniroma1.it