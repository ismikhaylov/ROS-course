cmake_minimum_required(VERSION 2.8.3)
project(defender)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rocket)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(defender src/defender.cpp)
target_link_libraries(defender ${catkin_LIBRARIES})


