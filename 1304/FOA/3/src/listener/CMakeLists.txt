cmake_minimum_required(VERSION 2.8.3)
project(listener)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  turtlesim
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rob2_node src/rob2_node.cpp)
target_link_libraries(rob2_node ${catkin_LIBRARIES})

add_executable(rob2_broadcaster src/rob2_broadcaster.cpp)
target_link_libraries(rob2_broadcaster ${catkin_LIBRARIES})
