cmake_minimum_required(VERSION 2.8.3)
project(cleaner)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    gazebo_ros
)
catkin_package(
	DEPENDS gazebo_ros
)

include_directories(
 	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
	${SDFormat_INCLUDE_DIRS}
)

add_executable(alg src/main.cpp src/robot.cpp)
target_link_libraries(alg ${catkin_LIBRARIES})
