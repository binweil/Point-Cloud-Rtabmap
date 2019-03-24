cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
PROJECT(pointcloud_task)

FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
FIND_PACKAGE(RTABMap 0.17.6 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)

catkin_package()
#Find Qt5 first
FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui Svg QUIET)
#IF(NOT Qt5_FOUND)
#   FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtSvg)
##ENDIF(NOT Qt5_FOUND)

# fix libproj.so not found on Xenial
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
   list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()

##if(POLICY CMP0020)
##   cmake_policy(SET CMP0020 OLD)
##endif()

# include directories
SET(INCLUDE_DIRS
    ${RTABMap_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
#/usr/local/include/rtabmap-0.18
)

#IF(QT4_FOUND)
#    INCLUDE(${QT_USE_FILE})
##ENDIF(QT4_FOUND)

INCLUDE_DIRECTORIES(
    ${INCLUDE_DIRS}
    /src
    /include
    )

# libraries
SET(LIBRARIES
    ${RTABMap_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${QT_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}

)

#IF(QT4_FOUND)
#    QT4_WRAP_CPP(moc_srcs MapBuilder.h)
#ELSE()
    QT5_WRAP_CPP(moc_srcs MapBuilder.h)
##ENDIF()

ADD_EXECUTABLE(rgbd_mapping src/main.cpp ${moc_srcs} ${INCLUDE_DIRECTORIES})
TARGET_LINK_LIBRARIES(rgbd_mapping ${LIBRARIES})
