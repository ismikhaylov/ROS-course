cmake_minimum_required(VERSION 2.8.3)
project(lab1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lab1 src/main.cpp)
target_link_libraries(lab1 ${catkin_LIBRARIES})
add_dependencies(lab1 lab1)

