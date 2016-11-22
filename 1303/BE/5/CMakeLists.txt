cmake_minimum_required(VERSION 2.8.3)
project(lab5)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  tf
  std_msgs
  gazebo_ros
)

catkin_package()

include_directories(
	  ${catkin_INCLUDE_DIRS}
  	  ${GAZEBO_INCLUDE_DIRS}
  	  ${SDFormat_INCLUDE_DIRS})

add_executable(R2D2 src/R2D2.cpp)
target_link_libraries(R2D2 ${catkin_LIBRARIES})

add_executable(C3Po src/C3Po.cpp)
target_link_libraries(C3Po ${catkin_LIBRARIES})