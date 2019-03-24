cmake_minimum_required(VERSION 2.8.3)
project(pcd_capture)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
FIND_PACKAGE(PCL 1.7 REQUIRED)
find_package(RTABMap 0.17.6 REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core Gui Svg QUIET)
find_package(VTK REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcd_capture
#  CATKIN_DEPENDS octomap roscpp rospy rtabmap std_msgs
#  DEPENDS system_lib
)

# include directories
SET(INCLUDE_DIRS
    ${RTABMap_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
#/usr/local/include/rtabmap-0.18
)

add_definitions(${PCL_DEFINITIONS})

include_directories(
# include
    ${INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# libraries
SET(LIBRARIES
    ${RTABMap_LIBRARIES}
    ${QT_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    Qt5::Widgets
)

add_executable(pcd_capture_node src/main.cpp src/viewer.cpp src/viewer.ui ${INCLUDE_DIRECTORIES})
target_link_libraries(pcd_capture_node ${LIBRARIES})
