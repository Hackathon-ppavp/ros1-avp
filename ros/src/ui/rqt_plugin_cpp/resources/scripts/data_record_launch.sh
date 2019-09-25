#!/bin/bash

# source ros system
source ~/.bashrc

# source ws
source ~/ppavp-datacapture/ros_ws/devel/setup.bash

# (read rosparam /save_destination set by ui) a bit hacky but will do for now
var=$(rosparam get /save_folder) 

# rosbag launch
roslaunch rqt_plugin_cpp data_record.launch file_folder:="$var"
