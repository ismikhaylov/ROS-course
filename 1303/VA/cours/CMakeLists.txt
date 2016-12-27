cmake_minimum_required(VERSION 2.8.3)
project(course)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  tf
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(wolf src/wolf.cpp)
target_link_libraries(wolf ${catkin_LIBRARIES})

add_executable(sheep src/sheep.cpp)
target_link_libraries(sheep ${catkin_LIBRARIES})

add_executable(stop src/stop.cpp)
target_link_libraries(stop ${catkin_LIBRARIES})
