cmake_minimum_required(VERSION 2.8.3)
project(publisher)

find_package(catkin REQUIRED COMPONENTS roscpp tf turtlesim)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(rob_node src/rob_node.cpp)
target_link_libraries(rob_node ${catkin_LIBRARIES})

add_executable(rob_broadcaster src/rob_broadcaster.cpp)
target_link_libraries(rob_broadcaster ${catkin_LIBRARIES})