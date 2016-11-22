cmake_minimum_required(VERSION 2.8.3)
project(robots)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(missing src/missing.cpp src/Robot.cpp)
target_link_libraries(missing ${catkin_LIBRARIES})

add_executable(assistant src/assistant.cpp src/Robot.cpp)
target_link_libraries(assistant ${catkin_LIBRARIES})