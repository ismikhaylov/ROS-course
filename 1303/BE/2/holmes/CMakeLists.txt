cmake_minimum_required(VERSION 2.8.3)
project(holmes)

find_package(catkin REQUIRED COMPONENTS
  place
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(holmes src/holmes.cpp)
target_link_libraries(holmes ${catkin_LIBRARIES})