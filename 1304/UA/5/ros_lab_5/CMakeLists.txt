cmake_minimum_required(VERSION 2.8.3)
project(ros_lab_5)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        std_msgs
        gazebo_ros
        )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

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
        #  INCLUDE_DIRS include
        #  LIBRARIES ros_lab_3
        #  CATKIN_DEPENDS other_catkin_pkg
          DEPENDS gazebo_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${SDFormat_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(ros_lab_5_ashen_one src/ashen_one.cpp)
add_executable(ros_lab_5_souls src/soul.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ros_lab_5_ashen_one ${catkin_LIBRARIES})
target_link_libraries(ros_lab_5_souls ${catkin_LIBRARIES})
