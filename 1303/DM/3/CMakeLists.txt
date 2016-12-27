cmake_minimum_required(VERSION 2.8.3)
project(packResc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	geometry_msgs
	visualization_msgs
	std_msgs
	tf)
	
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(lost src/lost.cpp)
target_link_libraries(lost ${catkin_LIBRARIES})

add_executable(rescuer src/rescuer.cpp)
target_link_libraries(rescuer ${catkin_LIBRARIES})


