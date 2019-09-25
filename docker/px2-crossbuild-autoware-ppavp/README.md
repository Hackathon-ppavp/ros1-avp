# Autoware Cross Build Docker
To use the cross build tool, first make sure Docker is properly installed.

[Docker installation](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/)

## How to Build Docker
Autoware users skip this step.
```
$ cd Autoware/docker/crossbuild/

# generic-aarch64
$ ./build_cross_image.sh generic-aarch64

# synquacer
$ ./build_cross_image.sh synquacer
```

## How to Build Cross
for driveworks, source cuda and nvidia toolkit
```
$ export DRIVEWORKS_TOOLKIT_ROOT_DIR=/usr/local/driveworks
$ export CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda
$ export NVIDIA_TENSORRT_ROOT_DIR=/usr/local/nvidia/tensorrt
$ export DRIVE_T186REF_LINUX_ROOT_DIR=/<directory to the toolkit>/drive-t186ref-linux
$ cd Autoware/ros/
# driveworks
$ ./catkin_make_release_cross driveworks
 
# generic-aarch64
$ ./colcon_release_cross generic-aarch64

# synquacer
$ ./colcon_release_cross synquacer
```
