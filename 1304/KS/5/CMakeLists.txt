cmake_minimum_required(VERSION 2.8.3)
project(lab5)

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

add_executable(ms src/missing.cpp src/robot.cpp)
target_link_libraries(ms ${catkin_LIBRARIES})

add_executable(as src/assistant.cpp src/robot.cpp)
target_link_libraries(as ${catkin_LIBRARIES})
