# Overview ppavp-sd 
This repository contains Autoware and ROS packages and other dependencies that are specific to the self-driving car development. The project builds on top of a pre-build Autoware docker image. The project also contains Gazebo version 8.  

The folders in 'ros' are organised as follow,  

**navigation**: anything to do with DBW system, motion control and path planning e.g. PID, waypoint loader, waypoint tracker  
**localisation_mapping**: system configuration and launch files for autoware ndt mapping   
**map_data**: any rosbag files and pcd (point cloud data) map  
**msgs**: custom messages  
**safety**: ultrasonic and lidar safety stop  
**sd_bringup**: main vehicle launch files that mix and match various launch files from autoware, ROS, and our own development package  
**simulation**: Gazebo simulation and urdf models  
**util**: additional utility tools e.g. convert ros bag to csv files and waypoint generation specific to this project  
**ui**: graphical user interface (work in progress)  
**visulization**: display data e.g. costmap plot

## System requirements
* GPU enabled machine
* Number of CPU cores: 8 or more
* RAM size: 32GB or more
* Recent version of [Docker CE](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
* [NVIDIA Docker v2](https://github.com/NVIDIA/nvidia-docker) 

## Installation using docker
Using the script to build ppavp-sd image  
```
$ cd docker
$ ./build-base.sh
```

Check docker image   
```
$ docker images
REPOSITORY       TAG                         IMAGE ID     CREATED      SIZE
ppavp-sd-test    base-cuda-1.12.0-kinetic    xxxx         xxxx         xxx 
```

TAG: base-cuda-[autoware version]-[ros vesrion]

## Run docker 
**start docker image**  
Note: we don't save the point cloud data inside the docker image. So you need to create the map data locally outside and mount it to docker using -v tag as following.

(Please create 'map_data' and 'autoware_openplanner_logs' (required by autoware open planner) folder before running the docker image)   
```
$ cd docker
$ ./run-ppavp-sd.sh -v 
OR
$ ./run-ppavp-sd.sh -v /home/$USER/map_data:/home/autoware/ppavp/src/map_data
OR
$ ./run-ppavp-sd.sh -v /home/$USER/map_data:/home/autoware/ppavp/src/map_data -v /home/$USER/autoware_openplanner_logs:/home/autoware/autoware_openplanner_logs
```

## Build our packages 
To build ROS packages once inside the docker, go into the following directory  
```
$ cd ppavp
$ catkin_make
```

## Getting start  
**Gazebo Simulation: Joy Stick Driving**   

'''
$ roslaunch car_demo test_gazebo.launch
'''


**Gazebo Simulation: Autonomous Driving**  

Fake localisation, WP loader, tracker, PurePursuit + PID  
NOTE: If you need to tune a better PID for steering and throttle, 
For Purepursuit approach see 'navigation/twist_to_control/launch/twist_to_pid_pp.launch'
For MPC approach (ONLY throttle required) see 'navigation/kinematics_to_control/launch/mpc_to_control.launch'


**[SAMPLE 1]:**
 
terminal 1: bringup the simulation  

```
$ roslaunch sim_bringup prius_wploader_pursuit_pid_mcity.launch
```

terminal 2: start navigation by publishing the enable command message once  
```
$ rostopic pub -1 /wp_enable std_msgs/Bool "data: true"
```

**[SAMPLE 2]:** 

terminal 1: bringup the simulation  

```
$  roslaunch sim_bringup prius_wploader_pursuit_pid_mcity_parking.launch
```

terminal 2: start navigation by publishing the enable command message once  
```
$ rostopic pub -1 /wp_enable std_msgs/Bool "data: true"
```

**[SAMPLE 3]:** 

terminal 1: bringup the simulation  

```
$  roslaunch sim_bringup prius_wploader_pursuit_pid_mcity_summon.launch
```

terminal 2: start navigation by publishing the enable command message once  
```
$ rostopic pub -1 /wp_enable std_msgs/Bool "data: true"


**[SAMPLE 4]:** 

terminal 1: bringup the simulation  

```
$  roslaunch sim_bringup prius_wploader_pursuit_pid_mcity_avp_demo.launch
```

terminal 2: start navigation by publishing the enable command message once  
```
$ rostopic pub -1 /route_csv std_msgs/String "data: 'park_mcity_path_1m.csv'"

then later

$ rostopic pub -1 /route_csv std_msgs/String "data: 'park_mcity_path_1m.csv'"


## Check lists
- Joy stick is connected to e.g. /dev/input/js0 if you need one to drive a car manually
















