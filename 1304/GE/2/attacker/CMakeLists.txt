cmake_minimum_required(VERSION 2.8.3)
project(attacker)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rocket)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(attacker src/attacker.cpp)
target_link_libraries(attacker ${catkin_LIBRARIES})


