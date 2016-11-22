cmake_minimum_required(VERSION 2.8.3)
project(lab1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(turtle_command src/turtle_command.cpp)
target_link_libraries(turtle_command ${catkin_LIBRARIES})
add_dependencies(turtle_command lab1_generate_messages_cpp)

add_executable(manipulator src/manipulator.cpp.cpp)
target_link_libraries(manipulator ${catkin_LIBRARIES})
add_dependencies(manipulator lab1_generate_messages_cpp)
