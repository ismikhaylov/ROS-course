cmake_minimum_required(VERSION 2.8.3)
project(reciever2)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs roc_message)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES architect_tutorials
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(reciveRocket src/reciveRocket.cpp)
target_link_libraries(reciveRocket ${catkin_LIBRARIES})

