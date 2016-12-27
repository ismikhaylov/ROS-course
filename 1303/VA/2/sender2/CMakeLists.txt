cmake_minimum_required(VERSION 2.8.3)
project(sender2)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs roc_message)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES architect_tutorials
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(sendRocket src/sendRocket.cpp)
target_link_libraries(sendRocket ${catkin_LIBRARIES})
