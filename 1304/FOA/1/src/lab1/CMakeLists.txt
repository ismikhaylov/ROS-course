cmake_minimum_required(VERSION 2.8.3)

project(lab1)

find_package(catkin REQUIRED 
COMPONENTS roscpp
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lab1.2 src/lab1_publisher.cpp)

target_link_libraries(lab1.2 ${catkin_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS roscpp)