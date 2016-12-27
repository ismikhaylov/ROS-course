cmake_minimum_required(VERSION 2.8.3)
project(lab2_comrade)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  lab2_transport
)

catkin_package()


include_directories(${catkin_INCLUDE_DIRS})

add_executable(lab2_comrade src/comrade.cpp)

target_link_libraries(lab2_comrade ${catkin_LIBRARIES})
