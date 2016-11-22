cmake_minimum_required(VERSION 2.8.3)
project(reader)

find_package(catkin REQUIRED COMPONENTS
  message
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(reader src/reader.cpp)
target_link_libraries(reader ${catkin_LIBRARIES})