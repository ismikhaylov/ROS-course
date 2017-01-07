cmake_minimum_required(VERSION 2.8.3)
project(l2publisher)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs l2message)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(l2publisher src/publisher.cpp)
target_link_libraries(l2publisher ${catkin_LIBRARIES})