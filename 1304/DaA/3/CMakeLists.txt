cmake_minimum_required(VERSION 2.8.3)
project(lab3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(first_bot src/first_main.cpp src/robot.cpp src/first_behavior.cpp)
add_executable(second_bot src/second_main.cpp src/robot.cpp src/second_behavior.cpp)

target_link_libraries(first_bot
  ${catkin_LIBRARIES}
)

target_link_libraries(second_bot
  ${catkin_LIBRARIES}
)