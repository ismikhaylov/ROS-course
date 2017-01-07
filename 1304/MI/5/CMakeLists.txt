cmake_minimum_required(VERSION 2.8.3)
project(twoRobots)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  gazebo_ros
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package(DEPENDS gazebo_ros)

include_directories(
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
  	${SDFormat_INCLUDE_DIRS}
)

add_executable(lost src/lostRobot.cpp)
add_executable(finder src/finderRobot.cpp)


target_link_libraries(lost
  ${catkin_LIBRARIES}
)

target_link_libraries(finder
  ${catkin_LIBRARIES}
)

