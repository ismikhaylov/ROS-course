cmake_minimum_required(VERSION 2.8.3)
project(robot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  tf
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(robot_gazebo src/robot_gazebo.cpp)
target_link_libraries(robot_gazebo ${catkin_LIBRARIES})
