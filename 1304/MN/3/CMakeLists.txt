cmake_minimum_required(VERSION 2.8.3)
project(lr3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  tf
  std_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lost src/LostRobot.cpp)
target_link_libraries(lost ${catkin_LIBRARIES})

add_executable(patrol src/PatrolRobot.cpp)
target_link_libraries(patrol ${catkin_LIBRARIES})
