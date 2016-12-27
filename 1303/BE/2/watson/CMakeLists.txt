cmake_minimum_required(VERSION 2.8.3)
project(watson)

find_package(catkin REQUIRED COMPONENTS
  place
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(watson src/watson.cpp)
target_link_libraries(watson ${catkin_LIBRARIES})