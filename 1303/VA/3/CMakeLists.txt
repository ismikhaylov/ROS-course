cmake_minimum_required(VERSION 2.8.3)
project(lab3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  tf
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(one src/one.cpp)
target_link_libraries(one ${catkin_LIBRARIES})

add_executable(two src/two.cpp)
target_link_libraries(two ${catkin_LIBRARIES})
