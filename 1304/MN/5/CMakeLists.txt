cmake_minimum_required(VERSION 2.8.3)
project(lab5)

set(CMAKE_CXX_FLAGS "-std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    std_msgs
    gazebo_ros
    message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   status.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_lab1
   CATKIN_DEPENDS roscpp tf std_msgs gazebo_ros message_runtime
   DEPENDS gazebo_ros
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

add_executable(lost_robot src/lost.cpp src/robot_info.cpp)
target_link_libraries(lost_robot ${catkin_LIBRARIES})

add_executable(finder_robot src/finder.cpp src/robot_info.cpp)
target_link_libraries(finder_robot ${catkin_LIBRARIES})

