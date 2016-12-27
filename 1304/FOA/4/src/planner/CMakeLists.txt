cmake_minimum_required(VERSION 2.8.3)
project(planner)


find_package(catkin REQUIRED COMPONENTS 
	roscpp
	tf
	sensor_msgs
	nav_msgs
	geometry_msgs

	)

catkin_package( )

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(planner src/planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} )