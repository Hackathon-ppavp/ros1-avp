cmake_minimum_required(VERSION 2.8.3)
project(twist_to_control)

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
        message_generation
        )

## Service
add_service_files(
        FILES
        brake.srv
        )

generate_messages(
        DEPENDENCIES
        std_msgs
        )

# catkin pkgs
catkin_package(CATKIN_DEPENDS
        sensor_msgs
        std_msgs
        geometry_msgs
        autoware_msgs
        tf
        message_runtime
        )

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        include
	       )


# simple twist to unit effort mapping
add_executable(twist_vehicle_mapper
        nodes/twist_vehicle_mapper.cpp
        )

add_dependencies(twist_vehicle_mapper
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(twist_vehicle_mapper
        ${catkin_LIBRARIES}
        )


# pid twist to unit effort controller
add_executable(twist_vehicle_pid
        nodes/twist_vehicle_pid.cpp
        nodes/dbw_pid.cpp
        )

add_dependencies(twist_vehicle_pid
        twist_to_control_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(twist_vehicle_pid
        ${catkin_LIBRARIES}
        )

# convert GLOBAL frame velocity to Vehicle frame velocity
add_executable(twist_velocity_base
        nodes/twist_velocity_base.cpp
        )

add_dependencies(twist_velocity_base
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(twist_velocity_base
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

install(TARGETS twist_vehicle_mapper twist_vehicle_pid twist_velocity_base
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
