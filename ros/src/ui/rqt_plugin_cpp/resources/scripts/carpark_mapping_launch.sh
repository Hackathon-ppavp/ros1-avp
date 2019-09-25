#!/bin/bash

# source ros system
source ~/.bashrc

# source ws
source ~/ppavp-datacapture/ros_ws/devel/setup.bash

# source Autoware workspace for pointcloud filter and ndt_mapping if NEEDED
#source /home/punnu/Autoware/ros/devel/setup.bash

# camera and lidar launch + optional mapping
roslaunch rqt_plugin_cpp carpark_mapping.launch
