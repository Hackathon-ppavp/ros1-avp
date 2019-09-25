# Build pcd map using ndt mapping  
In terminal 1: launch ndt mapping  
```
$ roslaunch ndt_localization carpark_mapping.launch 
```

In terminal 2: play bag file (~/ppavp/src/map_data/)  
```
$ cd ~/ppavp/src/map_data/
$ rosbag play turweston_carpark.bag /velodyne_points:=/points_raw
```

In terminal 3: save point clouds to pcd format  
```
$ rostopic pub --once /config/ndt_mapping_output autoware_msgs/ConfigNdtMappingOutput "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
filename: '/home/autoware/ppavp/src/map_data/mapping_test.pcd'
filter_res: 0.13" 
```


# Run localisation using ndt matching   
**Manual driving: bagfile**  
```
$ roslaunch sd_bringup sd_ndt_manual.launch
$ rosbag play turweston_carpark.bag
```

**Manual driving: live vlp 16 data**  
You may need to run this: sudo ifconfig enp<XX>s0 192.168.1.222 up  
```
$ roslaunch velodyne_pointcloud VLP16_points.launch device_ip:=192.168.1.201
$ roslaunch sd_bringup sd_ndt_manual.launch
```
(click 2D Pose Estimate in rviz if the car is not at 0,0)


**Autonomous DBW driving and wp loader**  
step 1: load all nodes  
```
$ roslaunch velodyne_pointcloud VLP16_points.launch device_ip:=192.168.1.201
$ roslaunch sd_bringup sd_ndt_autonomous.launch  
```
(in rviz use 2D Pose Estimate to set initial position)

step 2: start wp follower  
Note: use TAB for autocompletion
```
$ rostopic pub -1 /wp_enable <Tab for autocompletion>
```
