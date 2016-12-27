cmake_minimum_required(VERSION 2.8.3)
project(lab4node)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(lab4node src/lab4node.cpp)
target_link_libraries(lab4node ${catkin_LIBRARIES})

