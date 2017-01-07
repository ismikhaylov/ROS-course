cmake_minimum_required(VERSION 2.8.3)
project(moving_robot)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf
)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
add_executable(moving_robot src/moving_robot.cpp)
add_dependencies(moving_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(moving_robot
  ${catkin_LIBRARIES}
)
