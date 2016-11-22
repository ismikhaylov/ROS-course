cmake_minimum_required(VERSION 2.8.3)
project(lab2_receiver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  lab2_message
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lab2_receiver src/receiver.cpp)
add_dependencies(lab2_receiver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lab2_receiver ${catkin_LIBRARIES})
