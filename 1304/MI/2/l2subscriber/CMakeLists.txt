cmake_minimum_required(VERSION 2.8.3)
project(l2subscriber)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs l2message)

catkin_package()

include_directories(include  ${catkin_INCLUDE_DIRS})
add_executable(l2subscriber src/subscriber)
target_link_libraries(l2subscriber ${catkin_LIBRARIES})
