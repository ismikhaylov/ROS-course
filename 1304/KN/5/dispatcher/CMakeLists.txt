cmake_minimum_required(VERSION 2.8.3)
project(dispatcher)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  tf
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(dispatcher src/dispatcher.cpp)
target_link_libraries(dispatcher ${catkin_LIBRARIES})
