cmake_minimum_required(VERSION 2.8.3)
project(rqt_plugin_cpp)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -Wall -g ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rqt_gui
  rqt_gui_cpp
  std_msgs
)

## System dependencies are found with CMake's conventions
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()
roslint_cpp()

################################################
## Declare ROS messages, services and actions ##
################################################



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs
  #DEPENDS system_lib
)

set (rqt_plugin_cpp_SRCS
  src/rqt_plugin_cpp/my_plugin.cpp
  src/rqt_plugin_cpp/ros_interface.cpp
)

set(rqt_plugin_cpp_HDRS
  include/rqt_plugin_cpp/my_plugin.h
  include/rqt_plugin_cpp/ros_interface.h
)

set(rqt_plugin_cpp_UIS
  src/rqt_plugin_cpp/my_plugin.ui
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_plugin_cpp_MOCS ${rqt_plugin_cpp_HDRS})
  qt5_wrap_ui(rqt_plugin_cpp_UIS_H ${rqt_plugin_cpp_UIS})
else()
  qt4_wrap_cpp(rqt_plugin_cpp_MOCS ${rqt_plugin_cpp_HDRS})
  qt4_wrap_ui(rqt_plugin_cpp_UIS_H ${rqt_plugin_cpp_UIS})
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${rqt_plugin_cpp_SRCS}
  ${rqt_plugin_cpp_MOCS}
  ${rqt_plugin_cpp_UIS_H}
)

## Declare a C++ executable
# add_executable(rqt_example_cpp_node src/rqt_example_cpp_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(rqt_example_cpp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

#############
## Install ##
#############



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rqt_example_cpp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
