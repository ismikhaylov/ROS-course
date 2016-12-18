cmake_minimum_required(VERSION 2.8.3)
project(lab5)
find_package(catkin REQUIRED COMPONENTS roscpp tf visualization_msgs gazebo_ros)
catkin_package(DEPENDS gazebo_ros)
include_directories(
 	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
	${SDFormat_INCLUDE_DIRS}
)


add_executable(batman src/batman.cpp src/robot.cpp)
add_executable(robin src/robin.cpp src/robot.cpp)
target_link_libraries(batman ${catkin_LIBRARIES})
target_link_libraries(robin ${catkin_LIBRARIES})

