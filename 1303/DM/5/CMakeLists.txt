cmake_minimum_required(VERSION 2.8.3)
project(gRescue)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	geometry_msgs
	visualization_msgs
	std_msgs
	tf
	gazebo_ros)


catkin_package(
	DEPENDS gazebo_ros
)


include_directories(include ${catkin_INCLUDE_DIRS}
  							${GAZEBO_INCLUDE_DIRS}
  							${SDFormat_INCLUDE_DIRS})


add_executable(gLost src/gLost.cpp)
target_link_libraries(gLost ${catkin_LIBRARIES})

add_executable(gRescuer src/gRescuer.cpp)
target_link_libraries(gRescuer ${catkin_LIBRARIES})

add_executable(gTest src/gTest.cpp)
target_link_libraries(gTest ${catkin_LIBRARIES})


