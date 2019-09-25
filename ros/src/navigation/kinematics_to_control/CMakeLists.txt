cmake_minimum_required(VERSION 2.8.3)
project(kinematics_to_control)

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
        autoware_msgs
        tf
        )

# catkin pkgs
catkin_package(CATKIN_DEPENDS
        sensor_msgs
        std_msgs
        geometry_msgs
        autoware_msgs
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


# pid twist to unit effort controller
add_executable(kinematics_vehicle_pid
        nodes/kinematics_vehicle_pid.cpp
        )

add_dependencies(kinematics_vehicle_pid
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(kinematics_vehicle_pid
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

install(TARGETS kinematics_vehicle_pid
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
