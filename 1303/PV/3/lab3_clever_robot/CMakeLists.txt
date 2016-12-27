cmake_minimum_required(VERSION 2.8.3)
project(lab3_clever_robot)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   visualization_msgs
   tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lab3_clever_robot src/clever_robot.cpp)
target_link_libraries(lab3_clever_robot ${catkin_LIBRARIES})
add_dependencies(lab3_clever_robot lab3_clever_robot)
