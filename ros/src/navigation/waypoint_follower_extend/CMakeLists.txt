cmake_minimum_required(VERSION 2.8.3)
project(waypoint_follower_extend)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)

find_package(autoware_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        tf
        tf2
        geometry_msgs
        pcl_ros
        pcl_conversions
        sensor_msgs
        autoware_msgs
        amathutils_lib
        )

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp
        std_msgs
        tf
        tf2
        geometry_msgs
        autoware_msgs
        amathutils_lib
        pcl_ros
        pcl_conversions
        sensor_msgs
        gazebo_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${autoware_msgs_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

set(MPC_FOLLOWER_SRC
        nodes/mpc_follower/mpc_utils.cpp
        nodes/mpc_follower/mpc_trajectory.cpp
        nodes/mpc_follower/lowpass_filter.cpp
        nodes/mpc_follower/vehicle_model/vehicle_model_interface.cpp
        nodes/mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics.cpp
        nodes/mpc_follower/vehicle_model/vehicle_model_bicycle_dynamics.cpp
        nodes/mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.cpp
        nodes/mpc_follower/qp_solver/qp_solver_unconstr.cpp
        nodes/mpc_follower/qp_solver/qp_solver_unconstr_fast.cpp
)

add_executable(mpc_follower_extend nodes/mpc_follower/mpc_follower_node.cpp nodes/mpc_follower/mpc_follower_core.cpp ${MPC_FOLLOWER_SRC})
add_dependencies(mpc_follower_extend ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_follower_extend ${catkin_LIBRARIES})

add_executable(mpc_waypoints_converter_extend nodes/mpc_follower/mpc_waypoints_converter.cpp)
add_dependencies(mpc_waypoints_converter_extend ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_waypoints_converter_extend ${catkin_LIBRARIES})

## Install executables and/or libraries
install(TARGETS mpc_follower_extend mpc_waypoints_converter_extend
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
