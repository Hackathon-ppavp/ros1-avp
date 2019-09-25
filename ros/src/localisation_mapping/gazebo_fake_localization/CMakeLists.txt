cmake_minimum_required(VERSION 2.8.3)
project(gazebo_fake_localisation)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        geometry_msgs
        nav_msgs
        tf
        )

# catkin pkgs
catkin_package(CATKIN_DEPENDS
        sensor_msgs
        std_msgs
        geometry_msgs
        nav_msgs
        tf
        )

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        include
	       )

add_executable(groundtruth_tf_publisher
        nodes/groundtruth_tf_publisher.cpp
        nodes/groundtruth_tf.cpp
        )

add_dependencies(groundtruth_tf_publisher
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(groundtruth_tf_publisher
        ${catkin_LIBRARIES}
        )


#############
## Install ##
#############

install(TARGETS groundtruth_tf_publisher
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
