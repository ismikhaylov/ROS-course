cmake_minimum_required(VERSION 2.8.3)
project(search_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(lost_robot src/lost_robot.cpp src/Robot.cpp)
add_executable(finder_robot src/finder_robot.cpp src/Robot.cpp)

target_link_libraries(lost_robot ${catkin_LIBRARIES})
target_link_libraries(finder_robot ${catkin_LIBRARIES})
