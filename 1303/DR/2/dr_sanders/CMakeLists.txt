cmake_minimum_required(VERSION 2.8.3)
project(dr_sanders)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	roslib
	gn_tokugava
)

generate_messages()

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(dr_sanders src/main.cpp src/sanders.cpp)
target_link_libraries(dr_sanders ${catkin_LIBRARIES})